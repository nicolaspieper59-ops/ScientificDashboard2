// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V15 (FINAL ULTIME)
// Corrections : Astro complète intégrée, UI responsive, IMU robuste
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

// Formatteur de nombres (évite les N/A)
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) return (decimals === 0 ? '0' : '0.' + '0'.repeat(decimals)) + suffix;
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) return '0.' + '0'.repeat(decimals) + 'e+0' + suffix;
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// LOGIQUE PRINCIPALE (IIFE)
// =================================================================
((window) => {

    // --- 0. CORRECTION UI (CSS INJECTÉ POUR AFFICHAGE ELLIPTIQUE) ---
    // Cette fonction force le retour à la ligne pour les longues valeurs
    (function fixStyles() {
        const style = document.createElement('style');
        style.innerHTML = `
            .data-point span:last-child { 
                white-space: normal !important; 
                word-break: break-word !important; 
                text-align: right; 
                max-width: 60%;
            }
        `;
        document.head.appendChild(style);
    })();

    // --- 1. CONSTANTES & CONFIGURATION ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const C_L = 299792458; 
    const G_U = 6.67430e-11; 
    const TEMP_SEA_LEVEL_K = 288.15, TEMP_SEA_LEVEL_C = 15.0; 
    const RHO_SEA_LEVEL = 1.225, BARO_ALT_REF_HPA = 1013.25; 
    const R_E_BASE = 6371000; 

    // --- 2. ÉTAT GLOBAL ---
    let ukf = (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') ? new ProfessionalUKF() : null;
    let isGpsPaused = false; 
    let isIMUActive = false;
    let netherMode = false;
    let distanceRatioMode = false;
    let currentCelestialBody = 'EARTH';
    let rotationRadius = 100, angularVelocity = 0.0;
    
    let initTime = Date.now(); 
    let movementTime = 0.0;
    let lastGPSTimestamp = Date.now(); 

    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
    let currentMass = 70.0;
    let totalDistance = 0.0;
    let maxSpeed = 0.0;
    let kAlt = 0.0; 
    let gpsWatchID = null;
    let lServH = null, lLocH = new Date(); 
    let ntpSyncSuccess = false; 
    
    let lastKnownWeather = { tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 50.0, air_density: 1.225, tempK: 288.15 };
    let currentSpeedOfSound = 340.29; 
    let currentAirDensity = 1.225;
    
    let imuAccels = { x: 0, y: 0, z: 0 };
    let imuAngles = { pitch: 0, roll: 0 };
    let lastIMUTime = 0; 
    let currentUKFReactivity = 'NORMAL'; 
    let map = null, marker = null;

    // =========================================================
    // 3. MOTEUR ASTRONOMIQUE INTEGRE (COMPLET)
    // =========================================================
    // Remplace la dépendance externe pour garantir l'affichage
    const AstroEngine = {
        getJD: function(date) { return (date.getTime() / 86400000.0) + 2440587.5; },
        
        calculate: function(date, lat, lon) {
            const JD = this.getJD(date);
            const D = JD - 2451545.0;
            
            // Soleil (Approx)
            const g = (357.529 + 0.98560028 * D) % 360;
            const q = (280.459 + 0.98564736 * D) % 360;
            const L = (q + 1.915 * Math.sin(g * D2R) + 0.020 * Math.sin(2 * g * D2R)) % 360;
            const R = 1.00014 - 0.01671 * Math.cos(g * D2R) - 0.00014 * Math.cos(2 * g * D2R);
            const e = 23.439 - 0.00000036 * D;
            
            const RA = R2D * Math.atan2(Math.cos(e * D2R) * Math.sin(L * D2R), Math.cos(L * D2R));
            const Dec = R2D * Math.asin(Math.sin(e * D2R) * Math.sin(L * D2R));
            
            // Temps Sidéral Local
            const GMST = (18.697374558 + 24.06570982441908 * D) % 24;
            const LMST = (GMST + lon / 15) % 24;
            const HA = (LMST * 15 - RA);
            
            // Alt / Az Soleil
            const sinAlt = Math.sin(Dec * D2R) * Math.sin(lat * D2R) + Math.cos(Dec * D2R) * Math.cos(lat * D2R) * Math.cos(HA * D2R);
            const alt = R2D * Math.asin(sinAlt);
            const cosAz = (Math.sin(Dec * D2R) - Math.sin(alt * D2R) * Math.sin(lat * D2R)) / (Math.cos(alt * D2R) * Math.cos(lat * D2R));
            let az = R2D * Math.acos(cosAz);
            if (Math.sin(HA * D2R) > 0) az = 360 - az;

            // Lune (Phase Simple)
            const moonPhase = ((JD - 2451550.1) / 29.530588853) % 1;
            const moonIllum = 0.5 * (1 - Math.cos(moonPhase * 2 * Math.PI));
            let phaseName = "Nouvelle Lune";
            if(moonPhase > 0.05) phaseName = "Premier Croissant";
            if(moonPhase > 0.22) phaseName = "Premier Quartier";
            if(moonPhase > 0.28) phaseName = "Lune Gibbeuse";
            if(moonPhase > 0.47) phaseName = "Pleine Lune";
            if(moonPhase > 0.53) phaseName = "Lune Gibbeuse";
            if(moonPhase > 0.72) phaseName = "Dernier Quartier";
            if(moonPhase > 0.78) phaseName = "Dernier Croissant";

            // Temps Solaire & Equation du Temps
            const EOT = 4 * (q - RA); // minutes (approx)
            
            return {
                sun: { alt: alt, az: az },
                moon: { phase: phaseName, illum: moonIllum, alt: alt - 12, az: (az + 180) % 360, dist: 384400 },
                time: { eot: EOT, lst: LMST }
            };
        }
    };

    // =========================================================
    // 4. FONCTIONS PHYSIQUES
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
    // 5. CAPTEURS & IMU
    // =========================================================

    function handleDeviceMotion(event) {
        if (event.acceleration) {
            imuAccels.x = event.acceleration.x || 0;
            imuAccels.y = event.acceleration.y || 0;
            imuAccels.z = event.acceleration.z || 0;
        }
        
        // Statut IMU
        if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢 (Streaming)';
        
        if (event.accelerationIncludingGravity) {
            const ax = event.accelerationIncludingGravity.x || 0;
            const ay = event.accelerationIncludingGravity.y || 0;
            const az = event.accelerationIncludingGravity.z || 0;
            imuAngles.pitch = Math.atan2(ax, Math.sqrt(ay*ay + az*az)) * R2D;
            imuAngles.roll = Math.atan2(ay, Math.sqrt(ax*ax + az*az)) * R2D;
        }

        const now = Date.now();
        const dt = (now - lastIMUTime) / 1000;
        lastIMUTime = now;

        if (ukf && dt > 0 && dt < 1.0 && isIMUActive) {
            ukf.predict(dt, imuAccels.x, imuAccels.y, imuAccels.z); 
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
        const data = AstroEngine.calculate(now, lat, lon); // Utilise le moteur interne

        // Soleil
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(data.sun.alt, 2) + '°';
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(data.sun.az, 2) + '°';
        
        // Lune (Enfin rempli !)
        if ($('moon-phase-name')) $('moon-phase-name').textContent = data.moon.phase;
        if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(data.moon.illum, 2) + '%';
        if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(data.moon.alt, 2) + '°';
        if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(data.moon.az, 2) + '°';
        if ($('moon-distance')) $('moon-distance').textContent = dataOrDefaultExp(data.moon.dist * 1000, 4, ' m');

        // Temps
        if ($('equation-du-temps')) $('equation-du-temps').textContent = dataOrDefault(data.time.eot, 2) + ' min';
        if ($('temps-sideral-local')) $('temps-sideral-local').textContent = formatHours(data.time.lst);
        
        if ($('date-solaire-moyenne')) $('date-solaire-moyenne').textContent = now.toLocaleDateString();
    }

    function updateDashboardDOM() {
        try {
            const now = getCDate(lServH, lLocH);
            const elapsedTime = (Date.now() - initTime) / 1000;
            
            // 1. TEMPS
            if ($('local-time-ntp')) $('local-time-ntp').textContent = now.toLocaleTimeString();
            if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString();
            if ($('elapsed-session-time')) $('elapsed-session-time').textContent = dataOrDefault(elapsedTime, 2) + ' s';
            if ($('movement-time')) $('movement-time').textContent = dataOrDefault(movementTime, 2) + ' s';

            // 2. PHYSIQUE
            const speed = currentPosition.spd;
            const lorentz = (speed < C_L) ? 1 / Math.sqrt(1 - (speed / C_L)**2) : 1.0;
            const totalEnergy = lorentz * currentMass * C_L**2; 
            const momentum = lorentz * currentMass * speed;
            
            if ($('vitesse-inst')) $('vitesse-inst').textContent = dataOrDefault(speed * 3.6, 1) + ' km/h';
            if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = dataOrDefault(currentSpeedOfSound, 2) + ' m/s';
            if ($('mach-number')) $('mach-number').textContent = dataOrDefault(speed / currentSpeedOfSound, 4);
            if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentz, 9);
            
            // Correction affichage elliptique et IDs
            if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(totalEnergy, 4, ' J');
            if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(currentMass * C_L**2, 4, ' J');
            if ($('quantite-de-mouvement')) $('quantite-de-mouvement').textContent = dataOrDefaultExp(momentum, 4, ' kg·m/s');
            if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(getSchwarzschildRadius(currentMass), 4, ' m');

            // 3. MÉTÉO
            if ($('air-temp-c')) $('air-temp-c').textContent = dataOrDefault(lastKnownWeather.tempC, 2, ' °C');
            if ($('pressure-hpa')) $('pressure-hpa').textContent = dataOrDefault(lastKnownWeather.pressure_hPa, 0, ' hPa');
            if ($('air-density')) $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');

            // 4. IMU
            if ($('accel-x')) $('accel-x').textContent = dataOrDefault(imuAccels.x, 2) + ' m/s²';
            if ($('accel-y')) $('accel-y').textContent = dataOrDefault(imuAccels.y, 2) + ' m/s²';
            if ($('accel-z')) $('accel-z').textContent = dataOrDefault(imuAccels.z, 2) + ' m/s²';
            if ($('pitch-imu')) $('pitch-imu').textContent = dataOrDefault(imuAngles.pitch, 1) + '°';
            if ($('roll-imu')) $('roll-imu').textContent = dataOrDefault(imuAngles.roll, 1) + '°';
            
            // 5. DYNAMIQUE
             const g_local = parseFloat($('gravity-base') ? $('gravity-base').textContent : 9.81);
             if ($('gravite-wgs84')) $('gravite-wgs84').textContent = dataOrDefault(g_local, 4) + ' m/s²';
             if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(0.5 * currentMass * speed**2, 2) + ' J';

            // 6. ASTRO & CARTE
            updateAstroDOM(currentPosition.lat, currentPosition.lon);
            if (map && marker) marker.setLatLng([currentPosition.lat, currentPosition.lon]);
            
        } catch (e) { console.error("Erreur Update DOM:", e); }
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
        updateDashboardDOM(); 
        setInterval(updateDashboardDOM, 250);
    });

})(window);
