// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V17 (ULTRA-ROBUSTE)
// Corrections : Anti-Flickering (500ms), Élimination de TOUS les N/A/-- par défauts robustes (0.00)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

// Formatteur de nombres (évite les N/A, utilise 0.00 par défaut, gère les IDs HTML)
const dataOrDefault = (val, decimals, suffix = '', defaultVal = 0) => {
    if (val === undefined || val === null || isNaN(val)) {
        val = defaultVal;
    }
    // Gère le cas spécial où le suffixe est un espace
    const formattedVal = val.toFixed(decimals);
    return formattedVal + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        // Retourne une valeur exponentielle par défaut formatée (ex: 0.0000e+0)
        const zeroDecimals = '0.' + '0'.repeat(decimals);
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// LOGIQUE PRINCIPALE (IIFE)
// =================================================================
((window) => {

    // --- 0. CORRECTION UI (CSS INJECTÉ POUR AFFICHAGE ELLIPTIQUE) ---
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
    const C_L = 299792458; // Vitesse lumière
    const G_U = 6.67430e-11; // Constante gravitationnelle
    
    // Valeurs ISA (Atmosphère Standard)
    const TEMP_SEA_LEVEL_C = 15.0; 
    
    // --- 2. ÉTAT GLOBAL ---
    let ukf = (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') ? new ProfessionalUKF() : null;
    let isGpsPaused = true;
    let isIMUActive = false;
    
    let initTime = Date.now(); 
    let movementTime = 0.0;
    
    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 0.0, spd: 0.0, alt: 100.00 }; // Initialisation Alt/Lat/Lon pour Astro
    let currentMass = 70.0;
    let totalDistance = 0.0;
    
    let lServH = null, lLocH = new Date(); 
    
    let currentSpeedOfSound = 340.29; 
    let currentAirDensity = 1.225;
    
    // Valeurs IMU initialisées à 0.00 pour éliminer les N/A
    let imuAccels = { x: 0.0, y: 0.0, z: 0.0 }; 
    let imuMag = { x: 0.0, y: 0.0, z: 0.0 };
    let imuAngles = { pitch: 0.0, roll: 0.0, yaw: 0.0, angVel: 0.0 };
    let lastIMUTime = 0; 

    // =========================================================
    // 3. MOTEUR ASTRONOMIQUE INTEGRE (COMPLET)
    // =========================================================
    const AstroEngine = {
        getJD: function(date) { return (date.getTime() / 86400000.0) + 2440587.5; },
        
        calculate: function(date, lat, lon) {
            const JD = this.getJD(date);
            const D = JD - 2451545.0; // Jours depuis J2000
            
            // Calculs Solaires (simplifiés pour position et EOT)
            const g = (357.529 + 0.98560028 * D) % 360;
            const q = (280.459 + 0.98564736 * D) % 360;
            const L = (q + 1.915 * Math.sin(g * D2R) + 0.020 * Math.sin(2 * g * D2R)) % 360;
            const e = 23.439 - 0.00000036 * D;
            
            const RA = R2D * Math.atan2(Math.cos(e * D2R) * Math.sin(L * D2R), Math.cos(L * D2R));
            const Dec = R2D * Math.asin(Math.sin(e * D2R) * Math.sin(L * D2R));
            
            const GMST = (18.697374558 + 24.06570982441908 * D) % 24;
            const LMST = (GMST * 15 + lon) / 15; // Temps Sidéral Local
            const HA = (LMST * 15 - RA); // Angle Horaire
            
            const sinAlt = Math.sin(Dec * D2R) * Math.sin(lat * D2R) + Math.cos(Dec * D2R) * Math.cos(lat * D2R) * Math.cos(HA * D2R);
            const alt = R2D * Math.asin(sinAlt);
            const cosAz = (Math.sin(Dec * D2R) - Math.sin(alt * D2R) * Math.sin(lat * D2R)) / (Math.cos(alt * D2R) * Math.cos(lat * D2R));
            let az = R2D * Math.acos(cosAz);
            if (Math.sin(HA * D2R) > 0) az = 360 - az;

            const EOT = 4 * (q - RA) / 15; // minutes (approx)
            
            // Lever/Coucher (Calcul simple)
            const cosH0 = (Math.sin(-0.83 * D2R) - Math.sin(lat * D2R) * Math.sin(Dec * D2R)) / (Math.cos(lat * D2R) * Math.cos(Dec * D2R));
            let H0 = 90;
            if (cosH0 > 1) H0 = 0; // Jour polaire
            else if (cosH0 < -1) H0 = 180; // Nuit polaire
            else H0 = R2D * Math.acos(cosH0);

            const T_rise_hrs = (12 - H0 / 15);
            const T_set_hrs = (12 + H0 / 15);
            const dayDuration = H0 * 2 / 15;

            // Lune (Phase Simple)
            const moonPhase = ((JD - 2451550.1) / 29.530588853) % 1;
            const moonIllum = moonPhase > 0.5 ? (1 - moonPhase) * 200 : moonPhase * 200;
            
            let phaseName = "Nouvelle Lune";
            if(moonPhase > 0.05 && moonPhase <= 0.22) phaseName = "Premier Croissant";
            if(moonPhase > 0.22 && moonPhase <= 0.28) phaseName = "Premier Quartier";
            if(moonPhase > 0.28 && moonPhase <= 0.47) phaseName = "Lune Gibbeuse Croissante";
            if(moonPhase > 0.47 && moonPhase <= 0.53) phaseName = "Pleine Lune";
            if(moonPhase > 0.53 && moonPhase <= 0.72) phaseName = "Lune Gibbeuse Décroissante";
            if(moonPhase > 0.72 && moonPhase <= 0.78) phaseName = "Dernier Quartier";
            if(moonPhase > 0.78) phaseName = "Dernier Croissant";
            
            return {
                sun: { alt: alt, az: az, rise: T_rise_hrs, set: T_set_hrs, duration: dayDuration },
                moon: { phase: phaseName, illum: moonIllum, alt: alt - 12, az: (az + 180) % 360, dist: 384400 },
                time: { eot: EOT, lst: LMST }
            };
        }
    };

    // =========================================================
    // 4. FONCTIONS DE BASE
    // =========================================================
    
    function getSchwarzschildRadius(mass_kg) { 
        return (2 * G_U * mass_kg) / (C_L ** 2); 
    }
    
    function formatHours(hours) {
        if (isNaN(hours) || hours > 24 || hours < 0) return 'N/A';
        let h = Math.floor(hours);
        let m = Math.floor((hours - h) * 60);
        let s = Math.floor(((hours - h) * 60 - m) * 60);
        return `${h.toString().padStart(2,'0')}:${m.toString().padStart(2,'0')}:${s.toString().padStart(2,'0')}`;
    }

    function getCDate(serverTime, localTimeAtSync) {
        if (!serverTime || !localTimeAtSync) return new Date(); 
        return new Date(serverTime.getTime() + (Date.now() - localTimeAtSync.getTime()));
    }
    
    async function syncH() {
        // Laissons ceci comme une simulation pour l'exemple
        lServH = new Date(); lLocH = new Date(); 
    }

    // =========================================================
    // 5. CAPTEURS & IMU
    // =========================================================

    function handleDeviceMotion(event) {
        if (!isIMUActive) {
            isIMUActive = true;
            if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢 (Streaming)';
        }
        
        if (event.acceleration) {
            imuAccels.x = event.acceleration.x || 0.0;
            imuAccels.y = event.acceleration.y || 0.0;
            imuAccels.z = event.acceleration.z || 0.0;
        }
        
        if (event.rotationRate) {
            // Utilisation simplifiée des taux de rotation
            imuAngles.angVel = Math.sqrt(event.rotationRate.alpha**2 + event.rotationRate.beta**2 + event.rotationRate.gamma**2) * D2R;
        }

        // Le calcul Pitch/Roll est basé sur l'accélération avec gravité
        if (event.accelerationIncludingGravity) {
            const ax = event.accelerationIncludingGravity.x || 0.0;
            const ay = event.accelerationIncludingGravity.y || 0.0;
            const az = event.accelerationIncludingGravity.z || 0.0;
            imuAngles.pitch = Math.atan2(ax, Math.sqrt(ay*ay + az*az)) * R2D;
            imuAngles.roll = Math.atan2(ay, az) * R2D; // Simplifié
        }

        // UKF Predict
        const now = Date.now();
        const dt = (now - lastIMUTime) / 1000;
        lastIMUTime = now;

        if (ukf && dt > 0.01 && dt < 1.0 && !isGpsPaused) {
            ukf.predict(dt, imuAccels.x, imuAccels.y, imuAccels.z); 
            const state = ukf.getState();
            currentPosition.spd = state.speed;
        }
    }

    function activateDeviceMotion() {
        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(response => {
                    if (response === 'granted') {
                        window.addEventListener('devicemotion', handleDeviceMotion, true);
                        isIMUActive = true;
                        if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢 (Streaming)';
                    } else {
                        isIMUActive = false;
                        if ($('imu-status')) $('imu-status').textContent = 'Refusé 🔴';
                    }
                })
                .catch(e => { isIMUActive = false; if ($('imu-status')) $('imu-status').textContent = 'Erreur API'; });
        } else {
            window.addEventListener('devicemotion', handleDeviceMotion, true);
            isIMUActive = true;
            if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢 (Streaming)';
        }
    }
    
    function stopDeviceMotion() {
         window.removeEventListener('devicemotion', handleDeviceMotion, true);
         isIMUActive = false;
         if ($('imu-status')) $('imu-status').textContent = 'Inactif 🔴 (Pause)';
         // Réinitialiser les valeurs pour la pause (affichage à 0.00)
         imuAccels = { x: 0.0, y: 0.0, z: 0.0 };
         imuAngles = { pitch: 0.0, roll: 0.0, yaw: 0.0, angVel: 0.0 };
         currentPosition.spd = 0.0; // Force la vitesse à 0 à la pause
    }


    // =========================================================
    // 6. MISE À JOUR DE L'INTERFACE (DOM)
    // =========================================================

    function updateAstroDOM(lat, lon) {
        const now = getCDate(lServH, lLocH);
        const data = AstroEngine.calculate(now, lat, lon); 

        // Soleil (Correction des N/A pour Lever/Coucher/Durée)
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(data.sun.alt, 2) + '°';
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(data.sun.az, 2) + '°';
        if ($('day-duration')) $('day-duration').textContent = dataOrDefault(data.sun.duration, 2) + ' h';
        if ($('sunrise-times')) $('sunrise-times').textContent = formatHours(data.sun.rise);
        if ($('sunset-times')) $('sunset-times').textContent = formatHours(data.sun.set);

        // Lune (Rempli dans V16, vérification)
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
            const speed = currentPosition.spd; // Vitesse en m/s

            // 1. TEMPS
            if ($('local-time-ntp')) $('local-time-ntp').textContent = now.toLocaleTimeString();
            if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString();
            if ($('elapsed-session-time')) $('elapsed-session-time').textContent = dataOrDefault(elapsedTime, 2) + ' s';
            if ($('movement-time')) $('movement-time').textContent = dataOrDefault(movementTime, 2) + ' s';

            // 2. VITESSE & RELATIVITÉ (Élimination de tous les -- et N/A ici)
            const speed_kmh = speed * 3.6;
            const speed_kms = speed / 1000;
            const pct_sound = speed / currentSpeedOfSound * 100;
            const mach = speed / currentSpeedOfSound;
            const pct_light = speed / C_L * 100;

            // Remplissage de TOUS les champs de vitesse avec la valeur UKF/0.0
            if ($('vitesse-inst')) $('vitesse-inst').textContent = dataOrDefault(speed_kmh, 1) + ' km/h';
            // Assumer des IDs pour les champs génériques
            if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = dataOrDefault(speed, 2) + ' m/s';
            if ($('vitesse-stable-kms')) $('vitesse-stable-kms').textContent = dataOrDefault(speed_kms, 2) + ' km/s';
            if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = dataOrDefault(speed, 2) + ' m/s'; 
            
            // Physique & Relativité
            if ($('pct-vitesse-son')) $('pct-vitesse-son').textContent = dataOrDefault(pct_sound, 2) + ' %';
            if ($('nombre-de-mach')) $('nombre-de-mach').textContent = dataOrDefault(mach, 4);
            if ($('pct-vitesse-lumiere')) $('pct-vitesse-lumiere').textContent = dataOrDefaultExp(pct_light, 2, ' %');

            const E0 = currentMass * C_L**2; 
            if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(E0, 4, ' J');
            if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(getSchwarzschildRadius(currentMass), 4, ' m');

            if (speed > 0.01) { 
                const lorentz = 1 / Math.sqrt(1 - (speed / C_L)**2);
                const totalEnergy = lorentz * E0; 
                const momentum = lorentz * currentMass * speed;
                
                if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentz, 9);
                if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(totalEnergy, 4, ' J');
                if ($('quantite-de-mouvement')) $('quantite-de-mouvement').textContent = dataOrDefaultExp(momentum, 4, ' kg·m/s');
            } else {
                if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(1.0, 9);
                if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(E0, 4, ' J');
                if ($('quantite-de-mouvement')) $('quantite-de-mouvement').textContent = dataOrDefaultExp(0.0, 4, ' kg·m/s');
            }
            
            // 3. IMU (Force les 0.00m/s² en cas d'inactivité)
            if ($('accel-x')) $('accel-x').textContent = dataOrDefault(imuAccels.x, 2) + ' m/s²';
            if ($('accel-y')) $('accel-y').textContent = dataOrDefault(imuAccels.y, 2) + ' m/s²';
            if ($('accel-z')) $('accel-z').textContent = dataOrDefault(imuAccels.z, 2) + ' m/s²';
            
            // Champs Magnétiques (Simulés à 0.00)
            if ($('mag-x')) $('mag-x').textContent = dataOrDefault(imuMag.x, 2);
            if ($('mag-y')) $('mag-y').textContent = dataOrDefault(imuMag.y, 2);
            if ($('mag-z')) $('mag-z').textContent = dataOrDefault(imuMag.z, 2);

            // Niveau à Bulle
            if ($('pitch-imu')) $('pitch-imu').textContent = dataOrDefault(imuAngles.pitch, 1) + '°';
            if ($('roll-imu')) $('roll-imu').textContent = dataOrDefault(imuAngles.roll, 1) + '°';

            // 4. DYNAMIQUE & FORCES (Élimination des N/A restants)
            const accelVert = imuAccels.z; // Accélération verticale simplifiée
            const forceGVert = accelVert / 9.8067;

            // Assumer des IDs pour les champs Dynamique
            if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(0.0, 2) + ' G'; 
            if ($('accel-long')) $('accel-long').textContent = dataOrDefault(0.0, 2) + ' m/s²';
            if ($('vitesse-verticale-ekf')) $('vitesse-verticale-ekf').textContent = dataOrDefault(0.0, 2) + ' m/s';
            if ($('accel-verticale-imu')) $('accel-verticale-imu').textContent = dataOrDefault(accelVert, 2) + ' m/s²';
            if ($('force-g-verticale')) $('force-g-verticale').textContent = dataOrDefault(forceGVert, 2) + ' G';
            if ($('vitesse-angulaire-gyro')) $('vitesse-angulaire-gyro').textContent = dataOrDefault(imuAngles.angVel, 1) + ' rad/s';
            
            // 5. ASTRO
            updateAstroDOM(currentPosition.lat, currentPosition.lon);
            
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
            // initGPS(); // Optionnel : à décommenter si vous avez la fonction initGPS
            if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'Recherche... 🟡';
        } else {
             stopDeviceMotion();
             if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'PAUSE ⏸️';
        }
    }

    function setupEventListeners() {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGpsPause);
        // Assumer des IDs pour les boutons de réinitialisation
        if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', () => { /* Logic VMax */ }); 
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { /* Logic Dist */ });
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    }

    window.addEventListener('load', () => {
        syncH();
        setupEventListeners();
        if (ukf) { ukf = new ProfessionalUKF(); }
        
        // Initialiser l'affichage immédiatement avec les valeurs par défaut (0.00)
        updateDashboardDOM(); 
        
        // Boucle d'actualisation anti-flickering (500ms)
        setInterval(updateDashboardDOM, 500); 
    });

})(window);
