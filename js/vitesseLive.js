let watchId = null;
let positionPrecedente = null;

export function demarrerVitesseLive() {
  if (watchId !== null) return;

  watchId = navigator.geolocation.watchPosition(pos => {
    const gps = {
      latitude: pos.coords.latitude,
      longitude: pos.coords.longitude,
      accuracy: pos.coords.accuracy,
      timestamp: pos.timestamp,
      speed: pos.coords.speed
    };

    const vitesse = gps.speed !== null
      ? gps.speed * 3.6
      : calculerVitesse(gps);

    if (vitesse >= 0 && vitesse < 300) {
      afficherVitesse(vitesse, gps.accuracy);
    }
  }, err => {
    console.error("Erreur GPS :", err);
    afficherVitesse(0, null);
  }, {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 10000
  });
}

export function arreterVitesseLive() {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    positionPrecedente = null;
  }
}

function calculerVitesse(gps) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }

  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
  const d = calculerDistance(gps, positionPrecedente);
  positionPrecedente = gps;

  return dt > 0 ? (d / dt) * 3.6 : 0;
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

function afficherVitesse(vitesse, precision) {
  const affichage = document.getElementById('vitesse-gps');
  const statut = document.getElementById('statut-gps');

  if (affichage) {
    affichage.textContent = `${vitesse.toFixed(2)} km/h`;
  }

  if (statut) {
    if (precision === null) {
      statut.textContent = "⚠️ GPS indisponible";
    } else if (precision < 20) {
      statut.textContent = "🛰️ Connexion céleste établie";
    } else {
      statut.textContent = "🌫️ Signal faible";
    }
  }
     }
        
