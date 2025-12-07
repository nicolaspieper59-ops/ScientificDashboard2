// =================================================================
// FICHIER : gnss-dashboard.js (LOGICIEL MAÎTRE)
// RÔLE : Contrôleur principal. Relie le HTML, le GPS, et les librairies (UKF/Astro).
// DÉPENDANCES : ukf-lib.js, astro.js, suncalc.js, leaflet.js
// =================================================================

(function(window) {
    'use strict';

    // --- CONFIGURATION ---
    const CONFIG = {
        IMU_FREQ_MS: 20,      // 50Hz (Boucle Rapide)
        SLOW_UPDATE_MS: 1000, // 1Hz (Boucle Lente: Météo, Heure, Astro)
        GPS_OPTIONS: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
        WEATHER_API_URL: "https://scientific-dashboard2.vercel.app/api/weather",
        TIME_API_URL: "https://worldtimeapi.org/api/utc"
    };

    // --- ÉTAT GLOBAL ---
    const state = {
        ukf: null,              // Instance du filtre de Kalman
        isRunning: false,       // État du système
        gpsId: null,            // ID du watchPosition
        loops: { fast: null, slow: null },
        position: { lat: null, lon: null, alt: 0, accuracy: 0 },
        imu: { ax: 0, ay: 0, az: 0 }, // Accélération brute
        lastTime: 0,
        distTotal: 0,
        speedMax: 0,
        ntpOffset: 0,           // Différence temps Serveur - Local
        env: 'NORMAL',          // Environnement sélectionné
        map: null, marker: null, circle: null // Leaflet
    };

    // --- UTILITAIRES DOM ---
    const $ = id => document.getElementById(id);
    const setTxt = (id, val) => { const el = $(id); if(el) el.textContent = val; };

    // =================================================================
    // 1. GESTION DU TEMPS & SYNC NTP
    // =================================================================
    
    async function syncTime() {
        setTxt('local-time', 'Synchronisation...');
        const t0 = performance.now();
        try {
            const res = await fetch(CONFIG.TIME_API_URL, { cache: "no-store" });
            if(!res.ok) throw new Error('API Error');
            const data = await res.json();
            const t1 = performance.now();
            const serverTime = new Date(data.utc_datetime).getTime();
            const rtt = t1 - t0;
            // Calcul de l'offset : (TempsServeur + RTT/2) - TempsLocalActuel
            state.ntpOffset = (serverTime + rtt / 2) - Date.now();
            console.log("Synchro NTP OK. Offset:", state.ntpOffset, "ms");
        } catch (e) {
            console.warn("Échec NTP, utilisation heure système.", e);
            state.ntpOffset = 0;
            setTxt('local-time', 'Mode Hors Ligne');
        }
    }

    function getPreciseTime() {
        return new Date(Date.now() + state.ntpOffset);
    }

    // =================================================================
    // 2. GESTION DES CAPTEURS (GPS & IMU)
    // =================================================================

    function setupSensors() {
        // --- IMU (Accéléromètre) ---
        if (window.DeviceMotionEvent) {
            window.addEventListener('devicemotion', (e) => {
                // Priorité à l'accélération linéaire (sans gravité) si dispo
                const acc = e.acceleration || e.accelerationIncludingGravity;
                if (acc) {
                    state.imu.ax = acc.x || 0;
                    state.imu.ay = acc.y || 0;
                    state.imu.az = acc.z || 0;
                    setTxt('imu-status', 'Actif 🟢');
                }
            });
        } else {
            setTxt('imu-status', 'Non Supporté 🔴');
        }
    }

    function toggleGPS() {
        if (state.isRunning) {
            // ARRÊT
            navigator.geolocation.clearWatch(state.gpsId);
            clearInterval(state.loops.fast);
            clearInterval(state.loops.slow);
            state.isRunning = false;
            setTxt('gps-status-dr', 'Arrêté');
            const btn = $('toggle-gps-btn');
            if(btn) { btn.textContent = '▶️ MARCHE GPS'; btn.style.background = '#28a745'; }
        } else {
            // DÉMARRAGE
            if (!state.ukf) {
                try {
                    state.ukf = new window.ProfessionalUKF(); // Utilise la librairie corrigée
                } catch(e) {
                    alert("Erreur critique: UKF non chargé. Vérifiez ukf-lib.js");
                    return;
                }
            }
            
            state.gpsId = navigator.geolocation.watchPosition(onGPSUpdate, onGPSError, CONFIG.GPS_OPTIONS);
            
            // Démarrage des boucles
            state.lastTime = performance.now();
            state.loops.fast = setInterval(fastLoop, CONFIG.IMU_FREQ_MS);
            state.loops.slow = setInterval(slowLoop, CONFIG.SLOW_UPDATE_MS);
            
            state.isRunning = true;
            setTxt('gps-status-dr', 'Acquisition...');
            const btn = $('toggle-gps-btn');
            if(btn) { btn.textContent = '⏸️ PAUSE GPS'; btn.style.background = '#ffc107'; }
        }
    }

    function onGPSUpdate(pos) {
        const { latitude, longitude, altitude, accuracy, speed } = pos.coords;
        
        // Mise à jour de l'état brut
        state.position = { lat: latitude, lon: longitude, alt: altitude, accuracy };
        
        // 1. Mise à jour UKF (Correction)
        // Calcul d'un bruit dynamique basé sur l'environnement et la précision
        let R_dyn = (accuracy * accuracy); 
        if(state.env === 'FOREST') R_dyn *= 2.5; // Exemple simple
        
        if (state.ukf) {
            state.ukf.update({
                latitude, longitude, altitude, speed
            }, R_dyn);
        }

        // 2. Mise à jour Carte (Leaflet)
        if (state.map && state.marker) {
            const ll = [latitude, longitude];
            state.marker.setLatLng(ll);
            state.circle.setLatLng(ll).setRadius(accuracy);
            state.map.setView(ll, 16); // Centrer la carte
        }

        setTxt('gps-precision', `${accuracy.toFixed(1)} m`);
        // Appeler la météo si c'est la première fois qu'on a une position
        if(state.position.lat && $('temp-air-2').textContent.includes('N/A')) {
            updateWeather();
        }
    }

    function onGPSError(err) {
        console.error("Erreur GPS:", err);
        setTxt('gps-precision', `Erreur ${err.code}`);
    }

    // =================================================================
    // 3. BOUCLES DE TRAITEMENT (CORE LOGIC)
    // =================================================================

    // --- BOUCLE RAPIDE (50Hz) : Prédiction & Physique ---
    function fastLoop() {
        if (!state.ukf) return;

        const now = performance.now();
        const dt = (now - state.lastTime) / 1000; // en secondes
        state.lastTime = now;

        if (dt <= 0) return;

        // 1. Prédiction UKF avec données IMU
        state.ukf.predict({
            accel: [state.imu.ax, state.imu.ay, state.imu.az],
            gyro: [0, 0, 0] // Placeholder si pas de gyro
        }, dt);

        // 2. Récupération de l'état estimé
        const estimated = state.ukf.getState(); // { lat, lon, alt, speed, ... }

        // 3. Calculs dérivés
        state.distTotal += estimated.speed * dt;
        if (estimated.speed > state.speedMax) state.speedMax = estimated.speed;

        // 4. Mise à jour UI Rapide
        // IMU
        setTxt('accel-x', state.imu.ax.toFixed(2));
        setTxt('accel-y', state.imu.ay.toFixed(2));
        setTxt('accel-z', state.imu.az.toFixed(2));
        
        // Vitesse & Position
        setTxt('speed-stable', (estimated.speed * 3.6).toFixed(1)); // km/h
        setTxt('speed-stable-ms', estimated.speed.toFixed(2));
        setTxt('speed-max', (state.speedMax * 3.6).toFixed(1));
        setTxt('distance-total-km', `${(state.distTotal/1000).toFixed(3)} km`);
        
        setTxt('lat-display', estimated.lat.toFixed(6));
        setTxt('lon-display', estimated.lon.toFixed(6));
        setTxt('alt-display', estimated.alt ? estimated.alt.toFixed(1) : '0.0');

        // Physique (Relativité simplifiée pour l'exemple)
        const c = 299792458;
        const percentC = (estimated.speed / c) * 100;
        setTxt('perc-speed-c', percentC.toExponential(2) + " %");
        
        // Énergie Cinétique (Ec = 0.5 * m * v²)
        const mass = parseFloat($('mass-input')?.value) || 70;
        const ec = 0.5 * mass * (estimated.speed ** 2);
        setTxt('kinetic-energy', ec.toFixed(1) + " J");
    }

    // --- BOUCLE LENTE (1Hz) : Météo, Astro, Horloge ---
    function slowLoop() {
        const now = getPreciseTime();

        // 1. Horloge
        setTxt('local-time', now.toLocaleTimeString());
        setTxt('date-display', now.toLocaleDateString());
        setTxt('time-minecraft', window.getMinecraftTime ? window.getMinecraftTime(now) : "N/A");

        // 2. Astro (Seulement si position valide)
        if (state.position.lat && window.getSolarTime && window.SunCalc) {
            const solarData = window.getSolarTime(now, state.position.lon);
            
            setTxt('tst', solarData.TST);
            setTxt('mst', solarData.MST);
            setTxt('eot', solarData.EOT + " min");
            setTxt('ecl-long', solarData.ECL_LONG + "°");

            // Données positionnelles (SunCalc)
            const sunPos = window.SunCalc.getPosition(now, state.position.lat, state.position.lon);
            const moonPos = window.SunCalc.getMoonPosition(now, state.position.lat, state.position.lon);
            const moonIllum = window.SunCalc.getMoonIllumination(now);

            setTxt('sun-alt', (sunPos.altitude * 180 / Math.PI).toFixed(1) + "°");
            setTxt('sun-azimuth', (sunPos.azimuth * 180 / Math.PI).toFixed(1) + "°");
            setTxt('moon-phase-name', window.getMoonPhaseName(moonIllum.phase));
            setTxt('moon-illuminated', (moonIllum.fraction * 100).toFixed(0) + "%");
            
            // État Jour/Nuit simple
            const status = sunPos.altitude > 0 ? "Jour (☀️)" : "Nuit (🌙)";
            setTxt('clock-status', status);
        }
    }

    // =================================================================
    // 4. API EXTERNES (MÉTÉO)
    // =================================================================
    async function updateWeather() {
        if (!state.position.lat) return;
        setTxt('weather-status', 'Chargement...');
        
        try {
            const url = `${CONFIG.WEATHER_API_URL}?lat=${state.position.lat}&lon=${state.position.lon}`;
            const res = await fetch(url);
            const data = await res.json();
            
            if (data.main) {
                setTxt('temp-air-2', (data.main.temp).toFixed(1) + " °C");
                setTxt('pressure-2', data.main.pressure + " hPa");
                setTxt('humidity-2', data.main.humidity + " %");
                setTxt('weather-status', 'ACTIF 🟢');
            }
        } catch (e) {
            setTxt('weather-status', 'Erreur API');
        }
    }

    // =================================================================
    // 5. INITIALISATION (DOM READY)
    // =================================================================
    function init() {
        console.log("Démarrage du GNSS Dashboard...");
        
        // 1. Init Carte Leaflet (si disponible)
        if (window.L && $('map')) {
            state.map = L.map('map').setView([0, 0], 2);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap'
            }).addTo(state.map);
            state.marker = L.marker([0, 0]).addTo(state.map);
            state.circle = L.circle([0, 0], {radius: 100}).addTo(state.map);
        }

        // 2. Setup Listeners
        $('toggle-gps-btn')?.addEventListener('click', toggleGPS);
        $('reset-all-btn')?.addEventListener('click', () => {
            if(confirm('Réinitialiser ?')) window.location.reload();
        });
        $('environment-select')?.addEventListener('change', (e) => state.env = e.target.value);

        // 3. Préparations
        setupSensors();
        syncTime();

        // Affichage initial
        setTxt('gps-status-dr', 'Prêt à démarrer');
    }

    // Attendre que le DOM soit chargé
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }

})(window);
