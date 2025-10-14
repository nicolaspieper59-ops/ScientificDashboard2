<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Cockpit Cosmique 🚀 V. FINALE (Vitesse Verticale Calculée)</title>
    <style>
        body {
            background-color: #1e1e1e;
            color: #00ff00;
            font-family: 'Consolas', 'Courier New', monospace;
            padding: 20px;
            font-size: 14px;
            line-height: 1.6;
            margin: 0;
        }
        .cockpit-section {
            border: 1px solid #005500;
            padding: 15px;
            margin-bottom: 20px;
            border-radius: 5px;
            box-shadow: 0 0 10px rgba(0, 255, 0, 0.2);
        }
        h2 {
            color: #00ffff;
            border-bottom: 2px solid #00ffff;
            padding-bottom: 5px;
            margin-top: 0;
        }
        .controls button {
            background-color: #005500;
            color: #00ff00;
            border: 1px solid #00ff00;
            padding: 8px 15px;
            cursor: pointer;
            margin-right: 10px;
            border-radius: 3px;
            margin-top: 10px;
        }
        .controls button:hover {
            background-color: #008800;
        }
        .data-label {
            display: inline-block;
            width: 250px;
        }
        .warning {
            color: #ffaa00;
            font-weight: bold;
        }
        #gamma-factor {
            color: #ff00ff;
            font-weight: bold;
        }
    </style>
</head>
<body>

    <div id="cockpit">
        <div class="cockpit-section">
            <h2>🚀 Vitesse & Navigation</h2>
            <div id="navigation-data">
                <span class="data-label">Temps de voyage :</span> <span id="time-s">0 s</span><br>
                <span class="data-label">GPS :</span> <span id="gps-status" class="warning">En attente...</span><br>
                <span class="data-label">Vitesse instantanée (Horizontale):</span> <span id="vitesse-inst">0 km/h</span><br>
                <span class="data-label">Vitesse verticale ($\text{V}_{\text{z}}$ - Taux montée):</span> <span id="vitesse-vert">0 m/s</span><br>
                <span class="data-label">Vitesse moyenne :</span> <span id="vitesse-moy">0 km/h</span><br>
                <span class="data-label">Vitesse max :</span> <span id="vitesse-max">0 km/h</span><br>
                <span class="data-label">Vitesse (m/s | mm/s) :</span> <span id="vitesse-ms">0 m/s</span> | <span id="vitesse-mms">0 mm/s</span><br>
                <span class="data-label">Facteur Lorentz ($\gamma$) :</span> <span id="gamma-factor">1.0000</span><br>
                <span class="data-label">% Lumière (Ratio | Précis) :</span> <span id="pourcent-lumiere">0%</span> | <span id="pourcent-lumiere-precise">0 c</span><br>
                <span class="data-label">Distance (km | m | mm) :</span> <span id="distance-km">0 km</span> | <span id="distance-m">0 m</span> | <span id="distance-mm">0 mm</span><br>
                <span class="data-label">Distance cosmique (s-lumière | al) :</span> <span id="distance-sl">0 s lumière</span> | <span id="distance-al">0 al</span>
            </div>
            <div class="controls">
                <button id="start-btn" onclick="toggleMovement(true)">▶️ Marche</button>
                <button id="stop-btn" onclick="toggleMovement(false)" disabled>⏹️ Arrêt</button>
                <button onclick="resetData()">⚙️ Réinitialiser</button>
                <span class="data-label" style="width: 150px;">🚫 Mode Souterrain :</span> OFF
            </div>
        </div>

        <div class="cockpit-section">
            <h2>💫 Médaillon Céleste</h2>
            <div id="celestial-data">
                <span class="data-label">Date :</span> <span id="date">--</span><br>
                <span class="data-label">Heure Solaire Vraie :</span> <span id="hsv">--</span><br>
                <span class="data-label">Heure Solaire Moyenne :</span> <span id="hsm">--</span><br>
                <span class="data-label">Équation du Temps :</span> <span id="edt">-- s</span><br>
                <span class="data-label">Longitude Solaire :</span> <span id="lon-solaire">--°</span><br>
                <span class="data-label">Durée Jour Solaire :</span> <span id="djs">--</span><br>
                <span class="data-label">Lever Soleil :</span> <span id="lever-soleil">--</span><br>
                <span class="data-label">Coucher Soleil :</span> <span id="coucher-soleil">--</span><br>
                <span class="data-label">Horloge Cosmique (Statut du Médaillon) :</span> <span id="horloge-cosmique">--</span><br>
                <span class="data-label">Étoile Polaire (N) 🌠 :</span> <span id="polaire">--</span><br>
                <span class="data-label">Phase Lune (Élément Magique) :</span> <span id="phase-lune">--</span><br>
                <span class="data-label">Culmination Lune :</span> <span id="culmination-lune">--</span>
            </div>
        </div>

        <div class="cockpit-section">
            <h2>🧭 Carte Cosmique</h2>
            <div id="map-data">
                <span class="data-label">Point de Rendez-vous (Lat, Lon, Alt):</span> <span id="rendez-vous">43.2965, 5.3698</span><br>
                <span class="data-label">🎯 Cible</span><br>
                <span class="data-label">Relèvement vers la cible :</span> N/A | Distance : N/A<br>
                <span class="data-label">Boussole (Nord Vrai) :</span> <span id="boussole">N/A</span>
            </div>
        </div>

        <div class="cockpit-section">
            <h2>🔋 Capteurs & ⚛️ Grandeurs</h2>
            <h3>Capteurs (PWA/API)</h3>
            <div id="sensors-data">
                <span class="data-label">Accéléromètre (m/s²):</span> <span id="accel-status" class="warning">N/A (Autorisation Requise)</span><br>
                <span class="data-label">Température :</span> <span id="temp">N/A</span><br>
                <span class="data-label">Son :</span> N/A<br>
                <span class="data-label">Gyro :</span> N/A<br>
                <span class="data-label">Magnétomètre :</span> N/A<br>
                <span class="data-label">Batterie :</span> <span id="batterie">N/A</span><br>
                <span class="data-label">Réseau :</span> API Externe
            </div>
            <h3>Grandeurs Calculées</h3>
            <div id="calculated-data">
                <span class="data-label">Pression Est. :</span> SL: <span id="pression-sl">1013.25 hPa</span> | Local: <span id="pression-local">N/A</span><br>
                <span class="data-label">Énergie cinétique Est. :</span> <span id="energie-cinetique">0 J</span><br>
                <span class="data-label">Jour/Nuit :</span> <span id="jour-nuit">--</span><br>
                <span class="data-label">Heure atomique (UTC) :</span> <span id="utc-time">-- UTC</span>
            </div>
        </div>

        <div class="cockpit-section">
            <h2>🛠️ Contrôle Scientifique</h2>
            <div id="control-data">
                ✅ Rituel cosmique : ON 🧪 Physique : ON ⚗️ Chimie : ON 🌿 SVT : ON
            </div>
        </div>
    </div>

    <script src="cockpit.js"></script>
</body>
</html>
