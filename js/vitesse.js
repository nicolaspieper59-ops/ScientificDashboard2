let positionPrecedente = null;

export function collecterGPS() {
  return new Promise(resolve => {
    navigator.geolocation.getCurrentPosition(pos => {
      resolve({
        latitude: pos.coords.latitude,
        longitude: pos.coords.longitude,
        accuracy: pos.coords.accuracy,
        timestamp: pos.timestamp
      });
    }, err => {
      console.error("Erreur GPS :", err);
      resolve({ latitude: 0, longitude: 0, accuracy: null, timestamp: Date.now() });
    }, {
      enableHighAccuracy: true,
      timeout: 10000,
      maximumAge: 0
    });
  });
}

export function calculerVitesse(gps) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }

  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
  const d = calculerDistance(gps, positionPrecedente);
  positionPrecedente = gps;

  return dt > 0 ? (d / dt) * 3.6 : 0; // km/h
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

export function afficherVitesse(vitesse) {
  const affichage = document.getElementById('vitesse-gps');
  if (affichage) {
    affichage.textContent = `${vitesse.toFixed(2)} km/h`;
  }
}
