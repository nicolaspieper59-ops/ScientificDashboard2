<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<title>Cockpit Cosmique 3D – Kalman Corrigé</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
  body { background: #111; color: #0ff; font-family: monospace; padding: 20px; }
  h1 { color: #ffd700; }
  button { background:#222; color:#0ff; border:2px solid #0ff; padding:10px; cursor:pointer; margin:5px; font-weight:bold; }
  #infos { margin-top: 15px; border-top: 1px solid #333; padding-top: 10px; }
  #infos div { margin:5px 0; }
</style>
</head>
<body>
<h1>🌌 Cockpit Cosmique 3D – Kalman Corrigé</h1>

<section>
  <button id="marche">▶️ Démarrer</button>
  <button id="arreter">⏹️ Arrêter</button>
  <button id="reset">🔄 Réinitialiser vitesse max</button>
</section>

<div id="infos">
  <h2>🚀 Vitesse & Distance</h2>
  <div id="temps">Temps écoulé : 0.00 s</div>
  <div id="vitesse-raw">Vitesse GPS brute : -- km/h</div>
  <div id="vitesse-kalman">Vitesse filtrée : -- km/h</div>
  <div id="vitesse-max">Vitesse max : 0.00 km/h</div>
  <div id="distance">Distance totale : 0.000 km</div>

  <h2>📍 Précision & Capteurs</h2>
  <div id="gps">Lat: -- | Lon: -- | Alt: -- m | Acc: -- m</div>
  <div id="accel-z">Accélération Z : 0.00 m/s²</div>
  <div id="kalman-gain">Gain de Kalman (K) : --</div>
  <div id="kalman-error">Erreur estimée (P) : --</div>
  <div id="avertissement">⚠️ Géolocalisation et capteurs nécessitent HTTPS</div>
</div>

<script src="cockpit.js"></script>
</body>
</html>
