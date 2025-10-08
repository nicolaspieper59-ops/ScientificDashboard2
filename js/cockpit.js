function obtenirGPS() {
  if (!navigator.geolocation) {
    document.getElementById('statutGPS').textContent = "Statut GPS: Géolocalisation non supportée ❌";
    return;
  }

  navigator.geolocation.getCurrentPosition(
    position => {
      const lat = position.coords.latitude.toFixed(4);
      const lon = position.coords.longitude.toFixed(4);
      document.getElementById('latitude').value = lat;
      document.getElementById('longitude').value = lon;
      document.getElementById('statutGPS').textContent = `Statut GPS: Position obtenue ✅ (${lat}, ${lon})`;
    },
    error => {
      document.getElementById('statutGPS').textContent = "Statut GPS: Échec de la géolocalisation ❌";
    }
  );
}

function calculerTempsSolaire() {
  const dateStr = document.getElementById('date').value;
  const timeStr = document.getElementById('heure').value;
  const longitude = parseFloat(document.getElementById('longitude').value);
  const decalage = parseFloat(document.getElementById('decalage').value);

  if (isNaN(longitude) || isNaN(decalage)) {
    document.getElementById('resultats').textContent = "⚠️ Veuillez entrer une longitude et un décalage valides.";
    return;
  }

  const date = new Date(`${dateStr}T${timeStr}:00`);
  const heureLegale = date.getHours() + date.getMinutes() / 60;
  const midiSolaire = 12 - (longitude / 15) + decalage;
  const ajustement = heureLegale - midiSolaire;

  let suggestion = "";
  if (ajustement > 0.5) suggestion = "-30 min";
  else if (ajustement < -0.5) suggestion = "+30 min";
  else suggestion = "Ajustement non nécessaire";

  document.getElementById('resultats').textContent =
    `📍 Longitude: ${longitude.toFixed(2)}°\n` +
    `🕒 Heure Légale: ${heureLegale.toFixed(2)}h\n` +
    `🌞 Midi Solaire Vrai: ${midiSolaire.toFixed(2)}h\n` +
    `🔧 Ajustement Solaire Suggéré: ${suggestion}`;
}
