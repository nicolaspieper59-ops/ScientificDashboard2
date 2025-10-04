function demarrerGPS() {
  const result = document.getElementById('result');

  if (!navigator.geolocation) {
    result.textContent = '🌐 Géolocalisation non disponible';
    return;
  }

  navigator.geolocation.getCurrentPosition(
    pos => {
      const c = pos.coords;
      result.innerHTML = `
        Latitude : ${c.latitude.toFixed(6)}<br>
        Longitude : ${c.longitude.toFixed(6)}<br>
        Précision : ${c.accuracy.toFixed(1)} m
      `;
    },
    err => {
      result.textContent = `Erreur GPS : ${err.message}`;
    },
    { enableHighAccuracy: true, timeout: 10000 }
  );
}
