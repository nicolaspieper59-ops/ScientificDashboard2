<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>GNSS SpaceTime Dashboard • ULTIMATE 3D</title>
    
    <script src="https://unpkg.com/suncalc@1.9.0/suncalc.js"></script>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" integrity="sha256-20n636rY4H2K/eW7/iE8d4239W6QG2R5Gv5iN9XwQ3w=" crossorigin=""></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script> <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" integrity="sha256-p4NxAoJBhIINfBIHUPz0M7mR1mZ9/3wPzSS6i8tD6U4=" crossorigin=""/>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0-beta3/css/all.min.css" crossorigin="anonymous" referrerpolicy="no-referrer" />
          
    <style>
        :root { --bg-light: #f4f7f6; --bg-dark: #121220; --panel-light: #ffffff; --panel-dark: #1e1e1e; --text-light: #333; --text-dark: #e0e0e0; --accent-color: #007bff; }
        body { font-family: 'Consolas', monospace; margin: 0; padding: 0; background-color: var(--bg-light); color: var(--text-light); transition: 0.5s; }
        body.dark-mode { background-color: var(--bg-dark); color: var(--text-dark); }
        
        .dashboard-container { display: grid; grid-template-columns: repeat(4, 1fr); gap: 15px; padding: 15px; max-width: 1920px; margin: auto; }
        .section { background-color: var(--panel-light); padding: 15px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.05); }
        body.dark-mode .section { background-color: var(--panel-dark); box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
        
        .section h2 { margin-top: 0; border-bottom: 2px solid var(--accent-color); padding-bottom: 10px; font-size: 1.1em; color: var(--accent-color); }
        .data-point { display: flex; justify-content: space-between; padding: 4px 0; border-bottom: 1px dotted #eee; font-size: 0.9em; }
        .data-point span:first-child { font-weight: bold; color: #555; }
        .data-point span:last-child { font-family: 'Courier New', monospace; font-weight: bold; color: #0056b3; text-align: right; }
        body.dark-mode .data-point span:first-child { color: #aaa; }
        body.dark-mode .data-point span:last-child { color: #66b0ff; }

        /* Globe 3D */
        #globe-3d-container { height: 300px; width: 100%; border-radius: 8px; margin-top: 10px; background-color: #000; overflow: hidden; position: relative; }
        
        /* Horloge Minecraft (Disque Rotatif) */
        #astro-clock-container { position: relative; width: 100%; height: 150px; background: linear-gradient(to bottom, #87ceeb 0%, #ffffff 100%); border-radius: 8px; overflow: hidden; margin-top: 10px; transition: background 1s; }
        body.dark-mode #astro-clock-container { background: linear-gradient(to bottom, #0a0a2a 0%, #2a2a5a 100%); }
        .celestial-disk { position: absolute; width: 200px; height: 200px; top: 50%; left: 50%; margin-left: -100px; margin-top: -100px; transform-origin: center center; transition: transform 0.1s linear; }
        .celestial-body { position: absolute; font-size: 2em; }
        #sun-icon { top: 10px; left: 50%; transform: translateX(-50%); color: #ffdd00; text-shadow: 0 0 10px rgba(255, 221, 0, 0.8); }
        #moon-icon { bottom: 10px; left: 50%; transform: translateX(-50%) rotate(180deg); color: #f0f0f0; text-shadow: 0 0 5px rgba(255, 255, 255, 0.5); }
        .horizon-line { position: absolute; bottom: 0; width: 100%; height: 50%; background-color: rgba(56, 118, 29, 0.8); transition: background 0.5s, opacity 0.5s; z-index: 10; }
        .horizon-line.xray { opacity: 0.3; } /* Mode Rayons X */

        /* Contrôles */
        .controls-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 10px; }
        button { padding: 8px; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; width: 100%; transition: 0.2s; }
        #toggle-gps-btn { background-color: #28a745; color: white; grid-column: 1 / -1; }
        #emergency-stop-btn { background-color: #dc3545; color: white; grid-column: 1 / -1; }
        #emergency-stop-btn.active { animation: pulse-red 1s infinite alternate; }
        @keyframes pulse-red { 0% { box-shadow: 0 0 0 0 rgba(220, 53, 69, 0.7); } 100% { box-shadow: 0 0 0 10px rgba(220, 53, 69, 0); } }
        .control-input-full { margin-top: 5px; }
        .control-input-full input, .control-input-full select { width: 96%; padding: 4px; }
        
        .grid-span-2 { grid-column: span 2; }
        .grid-span-4 { grid-column: span 4; }
    </style>
</head>
<body>
    <div class="dashboard-container">

        <div class="section">
            <h2><i class="fas fa-sliders-h"></i> Contrôles & Système</h2>
            <div class="data-point"><span>Heure Locale (NTP)</span><span id="local-time">N/A</span></div>
            <div class="data-point"><span>Date Locale (UTC)</span><span id="date-display">N/A</span></div>
            <div class="data-point"><span>Temps Session</span><span id="time-elapsed">0.00 s</span></div>
            <div class="data-point"><span>Temps Mouvement</span><span id="time-moving">0.00 s</span></div>
            <div class="data-point"><span>Heure Minecraft</span><span id="time-minecraft">00:00</span></div>
            <div class="data-point"><span>Rapport Nether</span><span id="nether-ratio-display">DÉSACTIVÉ (1:1)</span></div>
            
            <div class="controls-grid">
                <button id="toggle-gps-btn">▶️ MARCHE GPS</button>
                <button id="emergency-stop-btn">🛑 Arrêt d'urgence</button>
                <button id="toggle-night-mode">🌙 Mode Nuit</button>
                <button id="toggle-xray-btn">👁️ Rayons X</button>
                <button id="reset-dist-btn">📏 Reset Dist.</button>
                <button id="reset-max-btn">🚀 Reset V-Max</button>
                <button id="reset-all-btn">⚠️ RESET TOUT</button>
                <button id="capture-data-btn">💾 CAPTURER</button>
            </div>

            <div class="control-input-full">
                <label>Précision GPS Forcée (m)</label>
                <input type="number" id="gps-accuracy-override" value="0" step="1">
            </div>
            <div class="control-input-full">
                <label>Environnement</label>
                <select id="environment-select"></select>
            </div>
            <div class="control-input-full">
                <label>Masse (kg)</label>
                <input type="number" id="mass-input" value="70">
            </div>
            <div class="control-input-full">
                <label>Corps Céleste</label>
                <select id="celestial-body-select">
                    <option value="EARTH">Terre</option>
                    <option value="MOON">Lune</option>
                    <option value="MARS">Mars</option>
                    <option value="ROTATING">Station Spatiale</option>
                </select>
            </div>
        </div>

        <div class="section">
            <h2><i class="fas fa-tachometer-alt"></i> Vitesse & Distance</h2>
            <div class="data-point"><span>Vitesse Stable (EKF)</span><span id="speed-stable-ekf">0.0 km/h</span></div>
            <div class="data-point"><span>Vitesse Stable (m/s)</span><span id="speed-stable-ms">0.00 m/s</span></div>
            <div class="data-point"><span>Vitesse 3D (Inst.)</span><span id="speed-inst-3d">0.0 km/h</span></div>
            <div class="data-point"><span>Vitesse Max (Session)</span><span id="speed-max">0.0 km/h</span></div>
            <div class="data-point"><span>Vitesse Moyenne (Totale)</span><span id="speed-avg-total">0.0 km/h</span></div>
            <div class="data-point"><span>Vitesse Micro (µm/s)</span><span id="speed-micro">0 µm/s</span></div>
            
            <hr>
            <div class="data-point"><span>% Vitesse Son</span><span id="perc-speed-sound">0.00 %</span></div>
            <div class="data-point"><span>% Vitesse Lumière</span><span id="perc-speed-c">0.00e+0 %</span></div>
            <div class="data-point"><span>Mach</span><span id="mach-number">0.0000</span></div>
            <div class="data-point"><span>Lorentz (γ)</span><span id="lorentz-factor">1.0000</span></div>
            
            <hr>
            <div class="data-point"><span>Distance Totale (3D)</span><span id="distance-total-km">0.000 km</span></div>
            <div class="data-point"><span>Ratio Distance (MRF)</span><span id="mode-ratio">1.000</span></div>
            <div class="data-point"><span>Dist. Horizon (Max)</span><span id="horizon-distance">0.00 km</span></div>
            <div class="data-point"><span>Dist. (s-lumière)</span><span id="distance-light-s">0.00e+0 s</span></div>
            <div class="data-point"><span>Dist. (UA)</span><span id="distance-cosmic">0.00e+0 UA</span></div>
        </div>

        <div class="section">
            <h2><i class="fas fa-globe"></i> Globe & Astro (Minecraft)</h2>
            <div id="globe-3d-container"></div> <div id="astro-clock-container">
                <div class="celestial-disk" id="celestial-disk">
                    <div id="sun-icon" class="celestial-body">☀️</div>
                    <div id="moon-icon" class="celestial-body">🌙</div>
                </div>
                <div class="horizon-line" id="horizon-visual"></div>
            </div>
            <div style="text-align: center; font-size: 0.8em;" id="clock-status">Initialisation...</div>

            <hr>
            <div class="data-point"><span>Heure Solaire Vraie</span><span id="tst">N/A</span></div>
            <div class="data-point"><span>Midi Solaire (UTC)</span><span id="solar-noon-utc">N/A</span></div>
            <div class="data-point"><span>Équation du Temps</span><span id="eot-display">N/A</span></div>
            <div class="data-point"><span>Temps Sidéral Local</span><span id="tslv">N/A</span></div>
            <div class="data-point"><span>Phase Lune</span><span id="moon-phase">N/A</span></div>
            <div class="data-point"><span>Élévation Soleil</span><span id="sun-elevation">-- °</span></div>
        </div>

        <div class="section">
            <h2><i class="fas fa-flask"></i> Météo, Chimie & SVT</h2>
            <div class="data-point"><span>Temp. Air</span><span id="temp-air-2">N/A</span></div>
            <div class="data-point"><span>Pression Atm.</span><span id="pressure-2">N/A</span></div>
            <div class="data-point"><span>Humidité</span><span id="humidity-2">N/A</span></div>
            <div class="data-point"><span>Densité Air</span><span id="air-density">N/A</span></div>
            <div class="data-point"><span>Vitesse Vent</span><span id="wind-speed-ms">N/A</span></div>
            <div class="data-point"><span>Temp. Ressentie</span><span id="temp-feels-like">N/A</span></div>
            <div class="data-point"><span>Oxygène (O₂)</span><span id="o2-level">20.9 %</span></div>
            <div class="data-point"><span>CO₂ (Sim.)</span><span id="co2-level">N/A</span></div>
            <div class="data-point"><span>Ozone (DU)</span><span id="ozone-conc">N/A</span></div>
            <div class="data-point"><span>pH (Sim.)</span><span id="ph-level">N/A</span></div>
            <div class="data-point"><span>Niveau Sonore</span><span id="noise-level">N/A</span></div>
            <div class="data-point"><span>Champ Magnétique</span><span id="magnetic-field">N/A</span></div>
            <div class="data-point"><span>Lumière (Lux)</span><span id="light-level">N/A</span></div>

            <hr>
            <h2><i class="fas fa-atom"></i> Physique Avancée</h2>
            <div class="data-point"><span>Gravité Locale</span><span id="gravity-local">N/A</span></div>
            <div class="data-point"><span>Force de Casimir</span><span id="casimir-force">N/A</span></div>
            <div class="data-point"><span>Pression Radiation</span><span id="radiation-pressure">N/A</span></div>
            <div class="data-point"><span>Dilatation T. (Vit)</span><span id="time-dilation-speed">0.00 ns/j</span></div>
            <div class="data-point"><span>Force de Coriolis</span><span id="coriolis-force">0.00 N</span></div>
            <div class="data-point"><span>Reynolds</span><span id="reynolds-number">N/A</span></div>
        </div>

        <div class="section grid-span-2">
            <h2><i class="fas fa-satellite"></i> Position & EKF 21-États</h2>
            <div class="data-point"><span>Latitude (EKF)</span><span id="latitude-ekf">N/A</span></div>
            <div class="data-point"><span>Longitude (EKF)</span><span id="longitude-ekf">N/A</span></div>
            <div class="data-point"><span>Altitude (EKF MSL)</span><span id="altitude-ekf">N/A</span></div>
            <div class="data-point"><span>Alt. Géopotentielle</span><span id="geopotential-alt">N/A</span></div>
            <div class="data-point"><span>Cap (Yaw Fusion)</span><span id="heading-ekf">N/A</span></div>
            <div class="data-point"><span>Incertitude Vit.</span><span id="kalman-uncert">N/A</span></div>
            <div class="data-point"><span>Incertitude Alt.</span><span id="alt-uncertainty">N/A</span></div>
            <div class="data-point"><span>Précision GPS</span><span id="gps-precision">N/A</span></div>
            <div class="data-point"><span>Statut Système</span><span id="gps-status-dr">Arrêté</span></div>
        </div>

        <div class="section grid-span-2">
            <h2><i class="fas fa-microchip"></i> IMU & Dynamique</h2>
            <div class="data-point"><span>Accel X (Body)</span><span id="accel-x">0.00 m/s²</span></div>
            <div class="data-point"><span>Accel Y (Body)</span><span id="accel-y">0.00 m/s²</span></div>
            <div class="data-point"><span>Accel Z (Body)</span><span id="accel-z">0.00 m/s²</span></div>
            <div class="data-point"><span>Accel Long. (EKF)</span><span id="accel-long-hybrid">0.00 m/s²</span></div>
            <div class="data-point"><span>Force G Long.</span><span id="force-g-long">0.00 G</span></div>
            <div class="data-point"><span>Force G Vert.</span><span id="force-g-vertical">0.00 G</span></div>
            <div class="data-point"><span>Traînée</span><span id="drag-force">0.00 N</span></div>
            <div class="data-point"><span>Énergie Cinétique</span><span id="kinetic-energy">0.00 J</span></div>
        </div>

    </div>

    <script src="gnss-dashboard-full.js"></script>
</body>
</html>
