let watchId = null;
let positionPrecedente = null;

export function demarrerVitesseLive() {

  watchId = navigator.geolocation.watchPosition(
    pos => {
      const vitesse = pos.coords.speed !== null
        ? pos.coords.speed * 3.6 // conversion m/s → km/h
        : calculerVitesse(pos);

      if (vitesse >= 0 && vitesse < 300) {
        document.getElementById('vitesse-gps').textContent = vitesse.toFixed(1) + " km/h";
        document.getElementById('statut-gps').textContent = "✅ Signal GPS actif";
      }
    },
    err => {
      console.error("Erreur GPS :", err);
      document.getElementById('statut-gps').textContent = "⚠️ Erreur GPS : " + err.message;
    },
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
}

export function arreterVitesseLive() {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    positionPrecedente = null;
  }
}

function calculerVitesse(pos) {
  if (!positionPrecedente) {
    positionPrecedente = pos;
    return 0;
  }

  const dt = (pos.timestamp - positionPrecedente.timestamp) / 1000; // en secondes
  const d = calculerDistance(
    positionPrecedente.coords.latitude,
    positionPrecedente.coords.longitude,
    pos.coords.latitude,
    pos.coords.longitude
  );

  positionPrecedente = pos;

  return dt > 0 ? (d / dt) * 3.6 : 0; // m/s → km/h
}

function calculerDistance(lat1, lon1, lat2, lon2) {
  const R = 6371e3; // rayon Terre
  const φ1 = lat1 * Math.PI / 180;
  const φ2 = lat2 * Math.PI / 180;
  const Δφ = (lat2 - lat1) * Math.PI / 180;
  const Δλ = (lon2 - lon1) * Math.PI / 180;

  const a = Math.sin(Δφ / 2) ** 2 +
            Math.cos(φ1) * Math.cos(φ2) *
            Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c; // en mètres
}
  
