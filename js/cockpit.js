let watchId = null;
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

  distanceTotale = 0; vitesses = []; vmax = 0; prevPos = null;
  lastAstroFetch=0; lastMeteoFetch=0;

  watchId = navigator.geolocation.watchPosition(
    onPosition,
    onGeoError,
    { enableHighAccuracy:true, maximumAge:5000, timeout:20000 }
  );

  set("gps","Recherche position...");
  requestAnimationFrame(tick);
}

function stop() {
  if(watchId!==null){
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    set("gps","GPS arrêté");
  }
}

function resetAll() {
  stop();
  const ids = ["latitude","longitude","altitude","gps","gps-brut",
               "vitesse","vitesse-moy","vitesse-max","vitesse-ms",
               "pourcentage","distance","distance-cosmique",
               "culmination-soleil","heure-solaire-vraie","heure-solaire-moyenne",
               "equation-temps","lune-phase","lune-magnitude","lever-lune",
               "coucher-lune","culmination-lune","temperature","pression","humidite",
               "vent","nuages","pluie","neige","uv","qualite-air","ebullition"];
  ids.forEach(id=>set(id,`${id.replace(/-/g," ")} : --`));
  prevPos=null; distanceTotale=0; vitesses=[]; vmax=0;
  lastAstroFetch=0; lastMeteoFetch=0;
}

function onPosition(pos){
  const c = pos.coords;
  set("latitude",`Latitude : ${c.latitude.toFixed(6)}`);
  set("longitude",`Longitude : ${c.longitude.toFixed(6)}`);
  set("altitude",`Altitude : ${c.altitude!==null?c.altitude.toFixed(1):"--"} m`);
  set("gps",`GPS : précision ${c.accuracy.toFixed(1)} m`);
  set("gps-brut",`Précision GPS : ${c.accuracy.toFixed(1)} m`);

  if(prevPos){
    const dt = Math.max((pos.timestamp-prevPos.timestamp)/1000,0.001);
    const d = calculerDistance(prevPos,c);
    distanceTotale += d;

    const v = (d/dt)*3.6; // km/h
    vitesses.push(v);
    if(v>vmax) vmax=v;
    if(vitesses.length>1000) vitesses.shift();
    const moy = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;

    const mps = v/3.6;
    const mmps = mps*1000;
    const pctL = (mps/299792458*100).toExponential(2);
    const pctS = (mps/343*100).toFixed(2);

    set("vitesse",`Vitesse instantanée : ${v.toFixed(2)} km/h`);
    set("vitesse-moy",`Vitesse moyenne : ${moy.toFixed(2)} km/h`);
    set("vitesse-max",`Vitesse max : ${vmax.toFixed(2)} km/h`);
    set("vitesse-ms",`Vitesse : ${mps.toFixed(2)} m/s | ${Math.round(mmps)} mm/s`);
    set("pourcentage",`% Lumière : ${pctL}% | % Son : ${pctS}%`);
    set("distance",`Distance : ${distanceTotale.toFixed(1)} m`);
    set("distance-cosmique",`Distance cosmique : ${(distanceTotale/299792458).toFixed(6)} s lumière`);
  }

  prevPos = {...c, timestamp:pos.timestamp};

  const now = Date.now();
  if(now - lastMeteoFetch > 60000){ lastMeteoFetch = now; meteoCosmique(c.latitude,c.longitude); }
  if(now - lastAstroFetch > 60000){ lastAstroFetch = now; medaillonCeleste(c.latitude,c.longitude); }
}

function onGeoError(err){
  set("gps",`Erreur géolocalisation : ${err.message}`);
}

function calculerDistance(a,b){
  const R=6371e3;
  const φ1=a.latitude*Math.PI/180;
  const φ2=b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180;
  const Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const s=Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return 2*R*Math.atan2(Math.sqrt(s),Math.sqrt(1-s));
}

// Tick horloge
function tick(){
  const t = performance.now();
  set("horloge-minecraft",new Date().toLocaleTimeString());
  set("heure-atomique",`Heure atomique (UTC) : ${new Date().toISOString().split('T')[1].split('.')[0]}`);
  requestAnimationFrame(tick);
}

// Médaillon céleste
async function medaillonCeleste(lat, lon){
  try{
    const moonRes = await fetch(`https://moon-api.com/api/v1/moon?lat=${lat}&lon=${lon}`);
    const moon = await moonRes.json();
    set("lune-phase",`Lune phase : ${moon.phase_percentage??"--"}%`);
    set("lune-magnitude",`Lune magnitude : ${moon.magnitude??"--"}`);
    set("lever-lune",`Lever lune : ${moon.moonrise??"--"}`);
    set("coucher-lune",`Coucher lune : ${moon.moonset??"--"}`);
    set("culmination-lune",`Culmination lune : ${moon.culmination??"--"}`);
  }catch(e){console.error("Moon API",e);}
  try{
    const now = new Date().toISOString();
    const solarRes = await fetch(`https://api.le-systeme-solaire.net/rest/positions?lon=${lon}&lat=${lat}&elev=0&datetime=${now}`);
    const solar = await solarRes.json();
    const soleil = solar.positions?.find(p=>p.name==="Sun");
    if(soleil){
      set("culmination-soleil",`Culmination soleil : ${soleil.altitude??"--"}`);
      set("heure-solaire-vraie",`Heure solaire vraie : ${soleil.local_time??"--"}`);
      set("heure-solaire-moyenne",`Heure solaire moyenne : ${new Date().toLocaleTimeString()}`);
      set("equation-temps",`Équation du temps : ${soleil.equation_time??"--"}`);
    }
  }catch(e){console.error("Soleil API",e);}
}

// Météo cosmique
async function meteoCosmique(lat, lon){
  try{
    const url=`https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}&current_weather=true&hourly=temperature_2m,pressure_msl,relative_humidity_2m,windspeed_10m,cloudcover,precipitation,uv_index&timezone=auto`;
    const data = await fetch(url).then(r=>r.json());
    const w = data.current_weather||{};
    const h = data.hourly||{};
    const idx = 0;

    set("temperature",`Température : ${w.temperature??"--"} °C`);
    set("pression",`Pression : ${h.pressure_msl?.[idx]??"--"} hPa`);
    set("humidite",`Humidité : ${h.relative_humidity_2m?.[idx]??"--"}%`);
    set("vent",`Vent : ${w.windspeed??"--"} km/h`);
    set("nuages",`Nuages : ${h.cloudcover?.[idx]??"--"}%`);
    set("pluie",`Pluie : ${h.precipitation?.[idx]??0} mm`);
    set("neige",`Neige : -- mm`);
    set("uv",`Indice UV : ${h.uv_index?.[idx]??"--"}`);
    set("qualite-air",`Qualité air : --`);

    const p = parseFloat(h.pressure_msl?.[idx]??1013);
    set("ebullition",`Point d’ébullition : ${!isNaN(p)?(100-((1013-p)*0.03)).toFixed(2):"--"} °C`);
  }catch(e){console.error("Météo API",e);}
}
