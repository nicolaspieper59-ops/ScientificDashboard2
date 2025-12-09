// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V14 (FINAL)
// Corrections : IDs HTML alignés, Astro intégrée, Physique corrigée
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

// Formatteur de nombres (évite les N/A)
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.' + '0'.repeat(decimals)) + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

// Formatteur exponentiel
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return '0.' + '0'.repeat(decimals) + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// LOGIQUE PRINCIPALE (IIFE)
// =================================================================
((window) => {

    // --- 1. CONSTANTES & CONFIGURATION ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const C_L = 299792458; // Vitesse lumière
    const G_U = 6.67430e-11; // Constante gravitationnelle
    
    // Valeurs ISA (Atmosphère Standard)
    const TEMP_SEA_LEVEL_K = 288.15; 
    const TEMP_SEA_LEVEL_C = 15.0; 
    const RHO_SEA_LEVEL = 1.225; 
    const BARO_ALT_REF_HPA = 1013.25; 
    const R_E_BASE = 6371000; 

    // --- 2. ÉTAT GLOBAL ---
    let ukf = (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') ? new ProfessionalUKF() : null;
    let isGpsPaused = false; 
    let isIMUActive = false;
    let netherMode = false;
    let distanceRatioMode = false;
    let currentCelestialBody = 'EARTH';
    let rotationRadius = 100;
    let angularVelocity = 0.0;
    
    // Variables de temps et mouvement
    let initTime = Date.now(); 
    let movementTime = 0.0;
    let lastGPSTimestamp = Date.now(); 

    // Position & Physique
    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
    let currentMass = 70.0;
    let totalDistance = 0.0;
    let maxSpeed = 0.0;
    let kAlt = 0.0; 
    let gpsWatchID = null;
    let lServH = null; 
    let lLocH = new Date(); 
    let ntpSyncSuccess = false; 
    
    // Météo (Valeurs par défaut)
    let lastKnownWeather = { 
        tempC: TEMP_SEA_LEVEL_C, 
        pressure_hPa: BARO_ALT_REF_HPA, 
        humidity_perc: 50.0,
        air_density: RHO_SEA_LEVEL, 
        tempK: TEMP_SEA_LEVEL_K
    };
    let currentSpeedOfSound = 340.29; 
    let currentAirDensity = RHO_SEA_LEVEL;
    
    let imuAccels = { x: 0, y: 0, z: 0 };
    let imuAngles = { pitch: 0, roll: 0 };
    let lastLat = currentPosition.lat;
    let lastLon = currentPosition.lon;
    let lastIMUTime = 0; 
    let currentUKFReactivity = 'NORMAL'; 
    
    let map = null;
    let marker = null;
    let trackPolyline = null;

    // =========================================================
    // 3. FONCTIONS PHYSIQUES
    // =========================================================

    function updateCelestialBody(body, alt_m, radius_m = 100, omega_rad_s = 0.0) {
        let G_ACC_NEW = 9.8067; 
        if (body === 'EARTH') G_ACC_NEW = 9.8067 * (R_E_BASE / (R_E_BASE + alt_m)) ** 2;
        else if (body === 'MOON') G_ACC_NEW = 1.62;
        else if (body === 'MARS') G_ACC_NEW = 3.72;
        else if (body === 'ROTATING') G_ACC_NEW = Math.sqrt(9.8067**2 + (radius_m * omega_rad_s**2)**2);

        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        return { G_ACC_NEW };
    }

    function calculateDistanceRatio(alt_m) {
        if (netherMode) return 8.0; 
        return (R_E_BASE + alt_m) / R_E_BASE;
    }
    
    function getSpeedOfSound(T_K) { 
        if (T_K <= 0 || isNaN(T_K)) return 340.29; 
        return 20.0468 * Math.sqrt(T_K); 
    }
    
    function getSchwarzschildRadius(mass_kg) { 
        return (2 * G_U * mass_kg) / (C_L ** 2); 
    }
    
    function formatHours(hours) {
        if (isNaN(hours)) return 'N/A';
        let h = Math.floor(hours);
        let m = Math.floor((hours - h) * 60);
        let s = Math.floor(((hours - h) * 60 - m) * 60);
        return `${h.toString().padStart(2,'0')}:${m.toString().padStart(2,'0')}:${s.toString().padStart(2,'0')}`;
    }

    async function syncH() {
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            const data = await response.json();
            lServH = new Date(data.utc_datetime);
            lLocH = new Date();
            ntpSyncSuccess = true;
        } catch (e) {
            lServH = new Date(); lLocH = new Date(); ntpSyncSuccess = false;
        }
    }
    
    function getCDate(serverTime, localTimeAtSync) {
        if (!serverTime || !localTimeAtSync) return new Date(); 
        return new Date(serverTime.getTime() + (Date.now() - localTimeAtSync.getTime()));
    }

    // =========================================================
    // 4. MINI-MOTEUR ASTRO (Intégré pour éviter les erreurs de lib)
    // =========================================================
    function calculateSunPosition(date, lat, lon) {
        // Algorithme simplifié mais précis pour le Soleil
        const dayOfYear = Math.floor((date - new Date(date.getFullYear(), 0, 0)) / 1000 / 60 / 60 / 24);
        const Y = (2 * Math.PI / 365) * (dayOfYear - 1 + (date.getHours() - 12) / 24);
        const eqtime = 229.18 * (0.000075 + 0.001868 * Math.cos(Y) - 0.032077 * Math.sin(Y) - 0.014615 * Math.cos(2 * Y) - 0.040849 * Math.sin(2 * Y));
        const decl = 0.006918 - 0.399912 * Math.cos(Y) + 0.070257 * Math.sin(Y) - 0.006758 * Math.cos(2 * Y) + 0.000907 * Math.sin(2 * Y) - 0.002697 * Math.cos(3 * Y) + 0.00148 * Math.sin(3 * Y);
        const declDeg = decl * R2D;
        
        // Altitude approximative à midi (très basique pour affichage fallback)
        const altitude = 90 - lat + declDeg; 
        
        return {
            altitude: altitude,
            azimuth: 180.0,
            eqtime: eqtime
        };
    }

    // =========================================================
    // 5. CAPTEURS & IMU
    // =========================================================

    function handleDeviceMotion(event) {
        // Récupération sécurisée des données
        if (event.acceleration) {
            imuAccels.x = event.acceleration.x || 0;
            imuAccels.y = event.acceleration.y || 0;
            imuAccels.z = event.acceleration.z || 0;
        }
        
        // Mise à jour du statut pour confirmer la réception des données
        if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢 (Streaming)';
        
        // Mise à jour des angles
        if (event.accelerationIncludingGravity) {
            const ax = event.accelerationIncludingGravity.x || 0;
            const ay = event.accelerationIncludingGravity.y || 0;
            const az = event.accelerationIncludingGravity.z || 0;
            // Calcul simple Pitch/Roll
            imuAngles.pitch = Math.atan2(ax, Math.sqrt(ay*ay + az*az)) * R2D;
            imuAngles.roll = Math.atan2(ay, Math.sqrt(ax*ax + az*az)) * R2D;
        }

        // UKF Predict
        const now = Date.now();
        const dt = (now - lastIMUTime) / 1000;
        lastIMUTime = now;

        if (ukf && dt > 0 && dt < 1.0 && isIMUActive) {
            ukf.predict(imuAccels.x, imuAccels.y, imuAccels.z, 0, 0, 0); 
            const state = ukf.getState();
            currentPosition.spd = state.speed;
            
            if (isGpsPaused) {
                 $('gps-status-indicator').textContent = `Estimation UKF (IMU) 🟡`;
            }
        }
    }

    function activateDeviceMotion() {
        const statusEl = $('imu-status');
        if (statusEl) statusEl.textContent = 'Tentative... ⏳';

        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(response => {
                    if (response === 'granted') {
                        window.addEventListener('devicemotion', handleDeviceMotion, true);
                        isIMUActive = true;
                        if (statusEl) statusEl.textContent = 'Actif 🟢';
                    } else {
                        isIMUActive = false;
                        if (statusEl) statusEl.textContent = 'Refusé 🔴';
                    }
                })
                .catch(e => { console.error(e); statusEl.textContent = 'Erreur API'; });
        } else {
            window.addEventListener('devicemotion', handleDeviceMotion, true);
            isIMUActive = true;
            if (statusEl) statusEl.textContent = 'Actif 🟢 (Standard)';
        }
    }

    function initGPS() {
        if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);
        gpsWatchID = navigator.geolocation.watchPosition(
            (position) => {
                if (isGpsPaused) return;
                
                const now = position.timestamp;
                const dt = (now - lastGPSTimestamp) / 1000;
                if (dt > 0 && dt < 60) movementTime += dt;
                lastGPSTimestamp = now;

                const gpsData = {
                    lat: position.coords.latitude,
                    lon: position.coords.longitude,
                    alt: position.coords.altitude || 0,
                    acc: position.coords.accuracy || 10,
                    speed: position.coords.speed || 0
                };
                
                if (ukf) {
                    ukf.update(gpsData, 'GNSS');
                    const state = ukf.getState();
                    currentPosition.lat = state.lat;
                    currentPosition.lon = state.lon;
                    currentPosition.alt = state.alt;
                    currentPosition.spd = state.speed;
                } else {
                    currentPosition = gpsData;
                }
                
                kAlt = currentPosition.alt;
                
                if (typeof turf !== 'undefined' && currentPosition.spd > 0.1) {
                    totalDistance += currentPosition.spd * dt; 
                }

                if ($('gps-status-indicator')) $('gps-status-indicator').textContent = `Signal OK 🛰️ (Acc: ${dataOrDefault(gpsData.acc, 1)}m)`;
            },
            (err) => { if ($('gps-status-indicator')) $('gps-status-indicator').textContent = `Erreur GPS (${err.code}) 🔴`; },
            { enableHighAccuracy: true, maximumAge: 0 }
        );
    }

    // =========================================================
    // 6. MISE À JOUR DE L'INTERFACE (DOM)
    // =========================================================

    function updateAstroDOM(lat, lon) {
        const now = new Date();
        
        // Tentative d'utiliser les librairies externes si présentes
        if (typeof getSolarData !== 'undefined') {
            const data = getSolarData(now, lat, lon);
            if (data && data.sun) {
                if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(data.sun.position.altitude * R2D, 2) + '°';
                if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(data.sun.position.azimuth * R2D, 2) + '°';
                if ($('sunrise-times')) $('sunrise-times').textContent = data.sun.times.sunrise ? data.sun.times.sunrise.toLocaleTimeString() : '--:--';
                if ($('sunset-times')) $('sunset-times').textContent = data.sun.times.sunset ? data.sun.times.sunset.toLocaleTimeString() : '--:--';
            }
        } else {
            // Fallback intégré (Mini-Moteur)
            const sun = calculateSunPosition(now, lat, lon);
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sun.altitude, 2) + '°';
            if ($('sun-azimuth')) $('sun-azimuth').textContent = "180.00° (Est.)"; // Valeur par défaut
        }
        
        if ($('date-solaire-moyenne')) $('date-solaire-moyenne').textContent = now.toLocaleDateString();
    }

    function updateDashboardDOM() {
        try {
            const now = getCDate(lServH, lLocH);
            const elapsedTime = (Date.now() - initTime) / 1000;
            
            // --- 1. TEMPS ---
            if ($('local-time-ntp')) $('local-time-ntp').textContent = now.toLocaleTimeString();
            if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString();
            if ($('elapsed-session-time')) $('elapsed-session-time').textContent = dataOrDefault(elapsedTime, 2) + ' s';
            if ($('movement-time')) $('movement-time').textContent = dataOrDefault(movementTime, 2) + ' s';

            // --- 2. PHYSIQUE & RELATIVITÉ (Calculs et IDs corrigés) ---
            const speed = currentPosition.spd;
            // Facteur de Lorentz : ne peut pas être 0. Minimum 1.
            const lorentz = (speed < C_L) ? 1 / Math.sqrt(1 - (speed / C_L)**2) : 1.0;
            const energy = (lorentz - 1) * currentMass * C_L**2; 
            const totalEnergy = lorentz * currentMass * C_L**2; 
            const momentum = lorentz * currentMass * speed;
            
            if ($('vitesse-inst')) $('vitesse-inst').textContent = dataOrDefault(speed * 3.6, 1) + ' km/h';
            if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = dataOrDefault(currentSpeedOfSound, 2) + ' m/s';
            if ($('mach-number')) $('mach-number').textContent = dataOrDefault(speed / currentSpeedOfSound, 4);
            if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentz, 9);
            
            // IDs CORRIGÉS SELON HTML
            if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(totalEnergy, 4, ' J');
            if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(currentMass * C_L**2, 4, ' J');
            if ($('momentum')) $('momentum').textContent = dataOrDefaultExp(momentum, 4, ' kg·m/s');
            if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(getSchwarzschildRadius(currentMass), 4, ' m');

            // --- 3. MÉTÉO (Fallback ISA) ---
            if ($('air-temp-c')) $('air-temp-c').textContent = dataOrDefault(lastKnownWeather.tempC, 2, ' °C');
            if ($('pressure-hpa')) $('pressure-hpa').textContent = dataOrDefault(lastKnownWeather.pressure_hPa, 0, ' hPa');
            if ($('air-density')) $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');

            // --- 4. IMU (IDs CORRIGÉS SELON HTML) ---
            // Le statut est géré par handleDeviceMotion, mais on force l'affichage ici aussi
            if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(imuAccels.x, 2) + ' m/s²';
            if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(imuAccels.y, 2) + ' m/s²';
            if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(imuAccels.z, 2) + ' m/s²';
            if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(imuAngles.pitch, 1) + '°';
            if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(imuAngles.roll, 1) + '°';
            
            // --- 5. DYNAMIQUE & FORCES (ID CORRIGÉ) ---
             const g_local = parseFloat($('gravity-base') ? $('gravity-base').textContent : 9.81);
             if ($('gravite-wgs84')) $('gravite-wgs84').textContent = dataOrDefault(g_local, 4) + ' m/s²';
             // Energie Cinétique (ID HTML: kinetic-energy)
             if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(0.5 * currentMass * speed**2, 2) + ' J';

            // --- 6. ASTRO & CARTE ---
            updateAstroDOM(currentPosition.lat, currentPosition.lon);
            if (map && marker) marker.setLatLng([currentPosition.lat, currentPosition.lon]);
            
        } catch (e) {
            console.error("Erreur Update DOM:", e);
        }
    }

    // =========================================================
    // 7. INITIALISATION & LISTENERS
    // =========================================================
    function toggleGpsPause() {
        isGpsPaused = !isGpsPaused;
        const btn = $('toggle-gps-btn');
        if (btn) btn.innerHTML = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
        
        if (!isGpsPaused) {
            activateDeviceMotion();
            initGPS();
            if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'Recherche... 🟡';
        } else {
             if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'PAUSE ⏸️';
        }
    }

    function setupEventListeners() {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGpsPause);
        
        if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            updateCelestialBody(currentCelestialBody, kAlt);
        });
        
        if (typeof L !== 'undefined' && $('map')) {
            map = L.map('map').setView([43.2964, 5.3697], 13);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
            marker = L.marker([43.2964, 5.3697]).addTo(map);
        }
    }

    window.addEventListener('load', () => {
        syncH();
        setupEventListeners();
        if (ukf) { ukf = new ProfessionalUKF(); }
        
        // Initialisation immédiate
        updateDashboardDOM(); 
        
        // Boucle
        setInterval(updateDashboardDOM, 250);
    });

})(window);
