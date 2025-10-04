function demarrer() {
  const el = id => document.getElementById(id);
  if (!navigator.geolocation) {
    el("gps").textContent = "Géolocalisation non disponible";
    return;
  }

  let t0 = performance.now();
  let positionPrecedente = null;
  let distanceTotale = 0;
  let vitesses = [];
  let vmax = 0;

  navigator.geolocation.watchPosition(pos => {
    const c = pos.coords;
    const t = pos.timestamp;
    el("latitude").textContent = `Latitude : ${c.latitude.toFixed(6)}`;
    el("longitude").textContent = `Longitude : ${c.longitude.toFixed(6)}`;
    el("altitude").textContent = `Altitude : ${c.altitude?.toFixed(1) ?? "--"} m`;
    el("gps").textContent = `Précision GPS : ${c.accuracy?.toFixed(1) ?? "--"}%`;
    el("gps-brut").textContent = `Précision GPS : ${c.accuracy?.toFixed(1) ?? "--"}`;

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

      el("vitesse").textContent = `Vitesse instantanée : ${v.toFixed(2)} km/h`;
      el("vitesse-moy").textContent = `Vitesse moyenne : ${moy.toFixed(2)} km/h`;
      el("vitesse-max").textContent = `Vitesse max : ${vmax.toFixed(2)} km/h`;
      el("vitesse-ms").textContent = `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`;
      el("pourcentage").textContent = `% Lumière : ${pctL}% | % Son : ${pctS}%`;
      el("distance").textContent = `Distance : ${(distanceTotale / 1000).toFixed(3)} km | ${distanceTotale.toFixed(1)} m | ${(distanceTotale * 1000).toFixed(0)} mm`;
      el("distance-cosmique").textContent = `Distance cosmique : ${ds.toFixed(6)} s lumière | ${dal.toExponential(3)} al`;
    }
  });

  function tick() {
    const t = performance.now() - t0;
    document.getElementById("temps").textContent = `Temps : ${(t / 1000).toFixed(2)} s`;
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
    "temperature", "pression", "humidite", "vent", "nuages", "pluie", "neige",
    "uv", "qualite-air", "ebull
