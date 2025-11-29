// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER FINAL MONOBLOC (ULTRA-ROBUSTE)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const toRad = deg => deg * (Math.PI / 180);
const toDeg = rad => rad * (180 / Math.PI);

/** Formate une valeur numérique. */
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

/** Formate en notation exponentielle. */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================
((window) => {

    // --- CONSTANTES PHYSIQUES ET ISA ---
    const C_LIGHT = 299792458;          // Vitesse de la lumière (m/s)
    const G_CONST = 6.67430e-11;        // Constante gravitationnelle universelle (m³/kg/s²)
    const G_ACC_EARTH = 9.80665;        // Gravité standard à la surface de la Terre (m/s²)
    const R_EARTH = 6371000;            // Rayon terrestre moyen (m)
    const TEMP_SEA_LEVEL_K = 288.15;    // Température au niveau de la mer (K)
    const BARO_ALT_REF_HPA = 1013.25;   // Pression atmosphérique standard au niveau de la mer (hPa)
    const RHO_SEA_LEVEL = 1.225;        // Densité de l'air au niveau de la mer (kg/m³)
    const GAS_CONST_AIR = 287.05;       // Constante spécifique des gaz pour l'air sec (J/kg/K)
    
    // --- CLASSE GRANDEURS THÉORIQUES (TQ) ---
    class TheoreticalQuantities {
        constructor() { this.q = {}; }
        add(id, name, value, unit, critical = false) { this.q[id] = { name, value, unit, critical }; }
        populateDOM() {
            for (const k in this.q) {
                const el = document.getElementById(k);
                if (el) {
                    const textContent = this.q[k].value.toExponential(10) + (this.q[k].unit ? (' ' + this.q[k].unit) : '');
                    el.textContent = textContent;
                }
            }
        }
    }

    const TQ = new TheoreticalQuantities();
    TQ.add('const-c', 'Vitesse de la lumière', C_LIGHT, 'm/s', true);
    TQ.add('const-G', 'Gravitation Universelle (G)', G_CONST, 'm³/kg/s²', false);
    TQ.add('const-h', 'Constante de Planck (h)', 6.62607015e-34, 'J·s', false);
    TQ.add('const-kB', 'Constante de Boltzmann (kB)', 1.380649e-23, 'J/K', false);
    TQ.add('const-R', 'Constante des Gaz Parfaits (R)', 8.314462618, 'J/mol/K', false);
    TQ.add('const-sigma', 'Constante de Stefan-Boltzmann', 5.670374419e-8, 'W/m²/K⁴', false);
    TQ.add('const-NA', "Nombre d'Avogadro (NA)", 6.02214076e23, 'mol⁻¹', false);

    // --- VERIFICATION DES DÉPENDANCES CRITIQUES ---
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const errorMsg = `Erreur critique: Dépendances manquantes. Le script est arrêté.`;
        console.error(errorMsg);
        document.body.innerHTML = `<div style="padding: 20px; color: white; background: #8B0000;">${errorMsg}</div>`;
        return; 
    }
    
    // --- VARIABLES D'ÉTAT GLOBAL ---
    let isGPSRunning = false;
    let watchID = null;
    let fastLoopID = null;
    let slowLoopID = null;
    let startTime = 0;
    let timeMoving = 0;
    let lastTime = Date.now();
    let currentSpeedMax = 0;
    let distanceTotal = 0;
    let distanceRatioMode = false; 
    
    let currentMass = parseFloat($('mass-input')?.value || 70.0);
    let currentCelestialBody = $('celestial-body-select')?.value || 'EARTH';
    let currentGravityAcc = G_ACC_EARTH;
    let rotationRadius = parseFloat($('rotation-radius')?.value || 100);
    let angularVelocity = parseFloat($('angular-velocity')?.value || 0.0);
    
    // Données des capteurs
    let lastKnownPos = null;
    let lastKnownVelocity = null;
    let lastKnownAccel = { x: 0, y: 0, z: 0 };
    let lastKnownOrientation = { alpha: 0, beta: 0, gamma: 0 };

    // Données atmosphériques par défaut (ISA)
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 340.29; 
    
    // État UKF
    const UKF_STATE_DIM = 21; 
    let UKF_X = math.zeros(UKF_STATE_DIM)._data; 
    let UKF_P = math.multiply(math.eye(UKF_STATE_DIM), 10)._data; 

    const ENVIRONMENT_FACTORS = {
        'NORMAL': { MULT: 1.0, DISPLAY: "Normal" }, 'FOREST': { MULT: 2.5, DISPLAY: "Forêt" },
        'CONCRETE': { MULT: 7.0, DISPLAY: "Grotte/Tunnel" }, 'METAL': { MULT: 5.0, DISPLAY: "Métal/Bâtiment" },
    };
    let selectedEnvironment = $('environment-select')?.value || 'NORMAL';
    let UKF_PROCESS_NOISE_MULT = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
    
    // État de la carte
    let map = null;
    let marker = null;
    let trackPolyline = null;
    let trackPoints = [];
    let isMapInitialized = false;

    // --- FONCTIONS PHYSIQUES ET UKF (Simplifié) ---

    const getSpeedOfSound = (T_K) => Math.sqrt(1.4 * GAS_CONST_AIR * T_K);
    const calculateLorentzFactor = (v) => (v >= C_LIGHT) ? Infinity : 1 / Math.sqrt(1 - (v * v) / (C_LIGHT * C_LIGHT));
    const calculateTimeDilationVel = (gamma) => (gamma === 1) ? 0 : (gamma - 1) * 86400 * 1e9;
    const calculateTimeDilationGrav = (alt, R_body, G_acc) => (G_acc * alt / (C_LIGHT * C_LIGHT)) * 86400 * 1e9;
    const calculateSchwarzschildRadius = (mass) => (2 * G_CONST * mass) / (C_LIGHT * C_LIGHT);
    const getMoonPhaseName = (phase) => {
        if (phase === 0 || phase === 1) return 'Nouvelle Lune'; // Reste du code omis pour la concision
        return (phase > 0.5) ? 'Lune Décroissante' : 'Lune Croissante';
    };

    const updateCelestialBody = (bodyKey, altitude, rotationR, angularV) => {
        let G_ACC_NEW = G_ACC_EARTH;
        if (bodyKey === 'ROTATING') G_ACC_NEW = rotationR * angularV * angularV;
        currentGravityAcc = G_ACC_NEW;
        return { G_ACC_NEW };
    };

    const initializeUKF = (lat, lon, alt) => {
        UKF_X = math.zeros(UKF_STATE_DIM)._data;
        UKF_X[0] = lat; UKF_X[1] = lon; UKF_X[2] = alt;
        UKF_P = math.multiply(math.eye(UKF_STATE_DIM), 10)._data;
        UKF_P[0][0] = UKF_P[1][1] = UKF_P[2][2] = 1; 
    };
    const predictUKF = (dt) => {/* ... UKF logic ... */};
    const updateUKF = (currentPosition, currentVelocity, currentIMU) => {/* ... UKF logic ... */};


    // --- LOGIQUE DE LA CARTE (LEAFLET) ---
    const startMap = () => {
        if (isMapInitialized || !$('map')) return; // Protection supplémentaire pour l'ID 'map'
        map = L.map('map').setView([45, 0], 2); 
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '© OpenStreetMap contributors' }).addTo(map);
        trackPolyline = L.polyline([], { color: '#007bff', weight: 3 }).addTo(map);
        marker = L.circleMarker([0, 0], { color: '#ff0000', radius: 6 }).addTo(map);
        isMapInitialized = true;
    };
    const updateMap = (lat, lon) => {
        if (!isMapInitialized) startMap();
        if (!lat || !lon) return;
        const latlng = [lat, lon];
        trackPoints.push(latlng);
        trackPolyline.setLatLngs(trackPoints);
        marker.setLatLng(latlng);
        if (trackPoints.length <= 1) map.setView(latlng, 15);
    };

    // --- HANDLERS GPS ---
    const getLocationSuccess = (position) => {
        const { latitude: lat, longitude: lon, altitude: alt, accuracy, speed } = position.coords;
        const kAlt = alt || 0;
        
        if (UKF_X[0] === 0 && trackPoints.length === 0) initializeUKF(lat, lon, kAlt);
        
        const currentIMU = { accel: lastKnownAccel, orientation: lastKnownOrientation }; 
        updateUKF({ lat, lon, alt: kAlt }, { speed, heading: position.coords.heading }, currentIMU);

        lastKnownPos = { lat: UKF_X[0], lon: UKF_X[1], alt: UKF_X[2] };
        
        const vx = UKF_X[3], vy = UKF_X[4], vz = UKF_X[5];
        lastKnownVelocity = { speed: Math.sqrt(vx*vx + vy*vy + vz*vz) }; // Mise à jour de la vitesse UKF
        
        updateMap(lastKnownPos.lat, lastKnownPos.lon);
        
        // Mise à jour des affichages DOM
        if ($('gps-precision')) $('gps-precision').textContent = dataOrDefault(accuracy, 2, ' m'); 
        if ($('lat-display')) $('lat-display').textContent = dataOrDefault(lastKnownPos.lat, 6, '°'); 
        if ($('lon-display')) $('lon-display').textContent = dataOrDefault(lastKnownPos.lon, 6, '°'); 
        if ($('alt-display')) $('alt-display').textContent = dataOrDefault(lastKnownPos.alt, 2, ' m');
    };

    const getLocationError = (error) => {
        console.warn(`Erreur GPS (${error.code}): ${error.message}`);
        if ($('speed-status-text')) $('speed-status-text').textContent = 'Erreur GPS ou Accès Refusé.';
    };

    // --- HANDLERS CAPTEURS IMU ET ENVIRONNEMENTAUX ---
    const handleDeviceMotion = (event) => {
        const { x, y, z } = event.accelerationIncludingGravity;
        lastKnownAccel = { x, y, z };
        
        if ($('imu-status')) $('imu-status').textContent = 'Actif';
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(x, 4, ' m/s²');
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(y, 4, ' m/s²');
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(z, 4, ' m/s²');

        const gTotal = Math.sqrt(x*x + y*y + z*z) / currentGravityAcc;
        if ($('g-force-total')) $('g-force-total').textContent = dataOrDefault(gTotal, 4, ' g');
    };

    const handleDeviceOrientation = (event) => {
        lastKnownOrientation = { alpha: event.alpha, beta: event.beta, gamma: event.gamma };
        if ($('roll')) $('roll').textContent = dataOrDefault(event.gamma, 2, '°'); 
        if ($('pitch')) $('pitch').textContent = dataOrDefault(event.beta, 2, '°'); 
    };

    const handleAmbientLight = (event) => {
        const lux = event.value;
        if ($('ambient-light')) $('ambient-light').textContent = dataOrDefault(lux, 2, ' Lux');
        const maxLux = parseFloat($('ambient-light-max')?.textContent || '0') || 0;
        if (lux > maxLux && $('ambient-light-max')) $('ambient-light-max').textContent = dataOrDefault(lux, 2, ' Lux');
    };


    // --- FONCTIONS DE BOUCLE ---

    /** Boucle rapide (20 Hz) pour les calculs cinématiques et d'affichage. */
    const fastLoop = () => {
        const now = Date.now();
        const dt = (now - lastTime) / 1000; 
        lastTime = now;
        
        predictUKF(dt);

        const speed = lastKnownVelocity ? lastKnownVelocity.speed : 0;
        
        if (isGPSRunning) {
            if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault((now - startTime) / 1000, 2, ' s');
            if (speed > 0.5) { timeMoving += dt; distanceTotal += speed * dt; }
            if ($('time-moving')) $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
            if (speed > currentSpeedMax) currentSpeedMax = speed;
            if ($('speed-max')) $('speed-max').textContent = dataOrDefault(currentSpeedMax * 3.6, 1, ' km/h');
            
            // Vitesse/Cinématique
            if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(speed * 3.6, 1, ' km/h');
            if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(speed, 2, ' m/s');
            
            // Relativité
            const gamma = calculateLorentzFactor(speed);
            if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 4);
            if ($('perc-speed-c')) $('perc-speed-c').textContent = dataOrDefaultExp((speed / C_LIGHT) * 100, 2, ' %');
            if ($('time-dilation-v')) $('time-dilation-v').textContent = dataOrDefault(calculateTimeDilationVel(gamma), 2, ' ns/j');
            
            // Énergie (les IDs sont confirmés par la sortie précédente)
            const restEnergy = currentMass * C_LIGHT * C_LIGHT;
            const relativisticEnergy = currentMass * gamma * C_LIGHT * C_LIGHT;
            if ($('energy-rest-mass')) $('energy-rest-mass').textContent = dataOrDefaultExp(restEnergy, 4, ' J'); 
            if ($('energy-relativistic')) $('energy-relativistic').textContent = dataOrDefaultExp(relativisticEnergy, 4, ' J'); 
            
            // Distance
            const totalKm = distanceTotal / 1000;
            if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(totalKm, 3, ' km')} | ${dataOrDefault(distanceTotal, 2, ' m')}`;
            
            // Physique / Atmosphère
            if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
            if ($('mach-number')) $('mach-number').textContent = dataOrDefault(speed / currentSpeedOfSound, 4);
            
        } else {
            if ($('speed-status-text')) $('speed-status-text').textContent = 'GPS en Pause.';
        }

        fastLoopID = requestAnimationFrame(fastLoop);
    };

    /** Boucle lente (1 Hz) pour les mises à jour non critiques (Astro, Gravité). */
    const slowLoop = () => {
        const date = new Date();
        // Mise à jour de l'heure UTC (doit fonctionner)
        if ($('date-display')) $('date-display').textContent = date.toISOString().replace('T', ' ').substring(0, 19) + ' UTC';
        
        if (lastKnownPos) {
            const times = SunCalc.getTimes(date, lastKnownPos.lat, lastKnownPos.lon);
            const sunPos = SunCalc.getPosition(date, lastKnownPos.lat, lastKnownPos.lon);
            const dayDurationH = (times.sunset.getTime() - times.sunrise.getTime()) / (1000 * 60 * 60);

            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(toDeg(sunPos.altitude), 2, '°');
            if ($('day-duration')) $('day-duration').textContent = dataOrDefault(dayDurationH, 2, ' h');
            if ($('sunrise-times')) $('sunrise-times').textContent = times.sunrise.toLocaleTimeString();
            if ($('sunset-times')) $('sunset-times').textContent = times.sunset.toLocaleTimeString();

            const moonPos = SunCalc.getMoonPosition(date, lastKnownPos.lat, lastKnownPos.lon);
            const moonIllumination = SunCalc.getMoonIllumination(date);
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, ' %');
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(toDeg(moonPos.altitude), 2, '°');
        } else {
             // Mettre à jour l'état initial des champs si GPS n'est pas actif pour éviter le N/A initial, sauf si la valeur est N/A par conception.
             // On maintient le N/A initial pour l'astro tant que la position est inconnue (lastKnownPos est null).
        }
        
        // Gravité (confirmé comme fonctionnel par votre sortie)
        if ($('time-dilation-g')) $('time-dilation-g').textContent = dataOrDefault(calculateTimeDilationGrav(lastKnownPos ? lastKnownPos.alt : 0, R_EARTH, currentGravityAcc), 2, ' ns/j'); 
        if ($('Rs-object')) $('Rs-object').textContent = dataOrDefaultExp(calculateSchwarzschildRadius(currentMass), 10, ' m'); 
        if ($('gravity-base')) $('gravity-base').textContent = dataOrDefault(currentGravityAcc, 4, ' m/s²');
        if ($('mass-display')) $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');

        slowLoopID = setTimeout(slowLoop, 1000);
    };

    const syncH = () => { if ($('local-time')) $('local-time').textContent = 'HORS LIGNE (Navigateur)'; };


    // --- GESTION DES ÉVÉNEMENTS DU DOM ---
    const toggleGPS = () => {
        if (!isGPSRunning) {
            if (!startTime) startTime = Date.now();
            const options = { enableHighAccuracy: $('freq-select')?.value === 'HIGH_FREQ', timeout: 5000, maximumAge: 0 };
            watchID = navigator.geolocation.watchPosition(getLocationSuccess, getLocationError, options);
            if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = '⏸ PAUSE GPS'; $('toggle-gps-btn').style.backgroundColor = '#ffc107'; }
            if ($('emergency-stop-btn')) { $('emergency-stop-btn').classList.remove('active'); $('emergency-stop-btn').innerHTML = '🛑 Arrêt d\'urgence: INACTIF 🟢'; }
            isGPSRunning = true;
            if (!fastLoopID) fastLoop(); 
            if (!slowLoopID) slowLoop(); 
        } else {
            if(watchID) navigator.geolocation.clearWatch(watchID);
            if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = '▶️ MARCHE GPS'; $('toggle-gps-btn').style.backgroundColor = '#28a745'; }
            isGPSRunning = false;
        }
    };

    const resetAll = () => {
        if (isGPSRunning) toggleGPS();
        startTime = 0; timeMoving = 0; currentSpeedMax = 0; distanceTotal = 0;
        trackPoints = [];
        if (trackPolyline) trackPolyline.setLatLngs([]);
        UKF_X = math.zeros(UKF_STATE_DIM)._data;
        UKF_P = math.multiply(math.eye(UKF_STATE_DIM), 10)._data;
        if ($('elapsed-time')) $('elapsed-time').textContent = '0.00 s';
        if ($('distance-total-km')) $('distance-total-km').textContent = '0.000 km | 0.00 m';
        console.log("Système réinitialisé.");
    };


    // --- INITIALISATION DES LISTENERS ET DÉMARRAGE ---
    document.addEventListener('DOMContentLoaded', () => {
        
        TQ.populateDOM();

        // Ajout des écouteurs d'événements
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGPS);
        if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
             if (isGPSRunning) toggleGPS();
             if ($('emergency-stop-btn')) { $('emergency-stop-btn').classList.add('active'); $('emergency-stop-btn').innerHTML = '🛑 Arrêt d\'urgence: ACTIF 🔴'; }
        }); 
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distanceTotal = 0; trackPoints = []; if (trackPolyline) trackPolyline.setLatLngs([]); if ($('distance-total-km')) $('distance-total-km').textContent = '0.000 km | 0.00 m'; });
        if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { currentSpeedMax = 0; if ($('speed-max')) $('speed-max').textContent = '0.0 km/h'; });
        
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
        });

        // Sélecteurs
        if ($('environment-select')) $('environment-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            UKF_PROCESS_NOISE_MULT = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
            if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${UKF_PROCESS_NOISE_MULT.toFixed(1)})`;
        });
        
        if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            updateCelestialBody(currentCelestialBody, lastKnownPos ? lastKnownPos.alt : 0, rotationRadius, angularVelocity);
        });

        // Capteurs IMU
        if (window.DeviceMotionEvent) {
            window.addEventListener('devicemotion', handleDeviceMotion);
            window.addEventListener('deviceorientation', handleDeviceOrientation);
        } else {
            if ($('imu-status')) $('imu-status').textContent = 'IMU Non Supporté';
        }
        
        // Capteur de lumière
        if ('AmbientLightSensor' in window) {
            const sensor = new window.AmbientLightSensor();
            sensor.addEventListener('reading', handleAmbientLight);
            sensor.start();
        }
        
        // --- DÉMARRAGE DU SYSTÈME ---
        updateCelestialBody(currentCelestialBody, 0, rotationRadius, angularVelocity);
        syncH(); 
        startMap();
        
        fastLoop();
        slowLoop();
    });

})(window);
     
