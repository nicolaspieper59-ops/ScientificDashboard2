function obtenirGPS() {
  if (navigator.geolocation) {
    navigator.geolocation.getCurrentPosition(position => {
      document.getElementById('latitude').value = position.coords.latitude.toFixed(4);
      document.getElementById('longitude').value = position.coords.longitude.toFixed(4);
      document.getElementById('statutGPS').textContent = "Statut GPS: Position obtenue ✅";
    }, () => {
      document.getElementById('statutGPS').textContent = "Statut GPS: Échec de la géolocalisation ❌";
    });
  } else {
    document.getElementById('statutGPS').textContent = "Statut GPS: Géolocalisation non supportée ❌";
  }
}

function calculerTempsSolaire() {
  const date = new Date(document.getElementById('date').value + 'T' + document.getElementById('heure').value + ':00');
  const longitude = parseFloat(document.getElementById('longitude').value);
  const decalage = parseFloat(document.getElementById('decalage').value);

  const midiSolaire = 12 - (longitude / 15) + decalage;
  const heureLegale = date.getHours() + date.getMinutes() / 60;
  const ajustement = heureLegale - midiSolaire;

  let suggestion = "";
  if (ajustement > 0.5) suggestion = "-30 min";
  else if (ajustement < -0.5) suggestion = "+30 min";
  else suggestion = "Ajustement non nécessaire";

  document.getElementById('resultats').textContent =
    `🕒 Midi Solaire Vrai estimé: ${midiSolaire.toFixed(2)}h\n` +
    `🔧 Ajustement Solaire Suggéré: ${suggestion}`;
}
