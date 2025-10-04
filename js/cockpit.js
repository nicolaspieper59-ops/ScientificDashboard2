function set(id, txt) {
  const el = document.getElementById(id);
  if (el) el.textContent = txt;
}

function demarrer() {
  if (!navigator.geolocation) return set("gps", "Géolocalisation non disponible");

  let t0 = performance.now();
  let positionPrecedente = null;
  let distanceTotale = 0;
  let vitesses = [];
  let vmax = 0;

  navigator.geolocation.watchPosition(pos => {
    const c = pos.coords;
    const t = pos.timestamp;
    set("latitude", `Latitude : ${c.latitude.toFixed(6)}`);
    set("longitude", `Longitude : ${c.longitude.toFixed(6)}`);
    set("altitude", `Altitude : ${c.altitude?.toFixed(1) ?? "--"} m`);
    set("gps", `GPS : ${c.accuracy?.toFixed(1) ?? "--"}%`);
    set("gps-brut", `Précision GPS : ${c.accuracy?.toFixed(1) ?? "--"}`);

    medaillonCeleste(c.latitude, c.longitude);
    meteoCosmique(c.latitude, c.longitude, c.altitude ?? 0);

    const dt = positionPrecedente ? (t - positionPrecedente.timestamp) / 1000 : 0;
    const d = positionPrecedente ? calculerDistance(c, positionPrecedente) : 0;
    distanceTotale += d;
    positionPrecedente = { ...c, timestamp: t };

    const v = dt > 0 ? (d / dt) * 3.6 : 0;
    if (v >= 0) {
      vitesses.push(v);
      vmax = Math.max(vmax, v);
      const moy = vitesses.reduce((a, b) => a + b, 0) / vitesses.length;
      const mps = v / 3.6;
      const mmps = mps * 1000;
      const ds = distanceTotale / 299792458;
      const dal = ds / (3600 * 24 * 365.25);
      const pctL = (mps / 299792458 * 100).toExponential(2);
      const pctS = (mps / 343 * 100).toFixed(2);

      set("vitesse", `Vitesse instantanée : ${v.toFixed(2)} km/h`);
      set("vitesse-moy", `Vitesse moyenne : ${moy.toFixed(2)} km/h`);
      set("vitesse-max", `Vitesse max : ${vmax.toFixed(2)} km/h`);
      set("vitesse-ms", `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
      set("pourcentage", `% Lumière : ${pctL}% | % Son : ${pctS}%`);
      set("distance", `Distance : ${(distanceTotale / 1000).toFixed(3)} km | ${distanceTotale.toFixed(1)} m | ${(distanceTotale * 1000).toFixed(0)} mm`);
      set("distance-cosmique", `Distance cosmique : ${ds.toFixed(6)} s lumière | ${dal.toExponential(3)} al`);
    }
  });

  function tick() {
    const t = performance.now() - t0;
    set("temps", `Temps : ${(t / 1000).toFixed(2)} s`);
    set("horloge-minecraft", new Date().toLocaleTimeString());
    set("heure-atomique", `Heure atomique (UTC) : ${new Date().toISOString().split("T")[1].split(".")[0]}`);
    requestAnimationFrame(tick);
  }
  tick();
}

function reset() {
  const ids = [
    "latitude", "longitude", "altitude", "gps", "gps-brut", "temps",
    "vitesse", "vitesse-moy", "vitesse-max", "vitesse-ms", "pourcentage",
    "distance", "distance-cosmique", "culmination-soleil", "heure-solaire-vraie",
    "heure-solaire-moyenne", "equation-temps", "lune-phase", "lune-magnitude",
    "lever-lune", "coucher-lune", "culmination-lune", "horloge-minecraft",
    "heure-atomique", "temperature", "pression", "humidite", "vent", "nuages",
    "pluie", "neige", "uv", "qualite-air", "ebullition"
  ];
  for (const id of ids) set(id, `${id.replace(/-/g, " ")} : --`);
}

function calculerDistance(a, b) {
  const R = 6371e3;
  const φ1 = a.latitude * Math.PI / 180;
  const φ2 = b.latitude * Math.PI / 180;
  const Δφ = (b.latitude - a.latitude) * Math.PI / 180;
  const Δλ = (b.longitude - a.longitude) * Math.PI / 180;
  const aVal = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(aVal), Math.sqrt(1 - aVal));
  return R * c;
}

async function medaillonCeleste(lat, lon) {
  try {
    const moonRes = await fetch(`https://moon-api.com/api/v1/moon?lat=${lat}&lon=${lon}`);
    const moon = await moonRes.json();
    set("lune-phase", `Lune phase : ${moon.phase_percentage}%`);
    set("lune-magnitude", `Lune magnitude : ${moon.magnitude}`);
    set("lever-lune", `Lever lune : ${moon.moonrise}`);
    set("coucher-lune", `Coucher lune : ${moon.moonset}`);
    set("culmination-lune", `Culmination lune : ${moon.culmination}`);
  } catch (e) {
    console.error("Moon API", e);
  }

  try {
    const now = new Date().toISOString();
    const solarRes = await fetch(`https://api.le-systeme-solaire.net/rest/positions?lon=${lon}&lat=${lat}&elev=0&datetime=${now}&zon=auto`);
    const solar = await solarRes.json();
    const soleil = solar.positions?.find(p => p.name === "Sun");
    if (soleil) {
      set("culmination-soleil", `Culmination soleil : ${soleil.altitude}`);
      set("heure-solaire-vraie", `Heure solaire vraie : ${soleil.local_time}`);
      set("heure-solaire-moyenne", `Heure solaire moyenne : ${new Date().toLocaleTimeString()}`);
      set("equation-temps", `Équation du temps : ${soleil.equation_time}`);
    }
  } catch (e) {
    console.error("Soleil API", e);
  }
}

async function meteoCosmique(lat, lon, alt = 0) {
  try {
    const url = `https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}&current_weather=true&hourly=temperature_2m,pressure_msl,relative_humidity_2m,windspeed_10m,cloudcover,precipitation,uv_index&timezone=auto`;
    const res = await fetch(url);
    const data = await res.json();
    const w = data.current_weather;

    set("temperature", `Température : ${w.temperature} °C`);
    set("pression", `Pression : ${w.pressure_msl ?? "--"} hPa`);
    set("humidite", `Humidité : ${data.hourly.relative_humidity_2m[0] ?? "--"}%`);
    set("vent", `Vent : ${w.windspeed} km/h`);
    set("nuages", `Nuages : ${data.hourly.cloudcover[0] ?? "--"}%`);
    set("pluie", `Pluie : ${data.hourly.precipitation[0] ?? "0"} mm`);
    set("neige", `Neige : -- mm`);
    set("uv", `Indice UV : ${data.hourly.uv_index[0] ?? "--"}`);
    set("qualite-air", `Qualité air : --`);

    const pression = w.pressure_msl ?? 1013;
    const ebullition = 100 - ((1013 - pression) * 0.03);
    set("ebullition", `Point d’ébullition : ${ebullition.toFixed
