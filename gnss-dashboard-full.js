// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/5 : Constantes, Utilitaires, UKF Class & État Global
// =================================================================
((window) => {

    // --- FONCTIONS UTILITAIRES GLOBALES ---
    const $ = id => document.getElementById(id);
    const dataOrDefault = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val) || val === Infinity) {
            return (decimals === 0 ? '0' : '0.00') + suffix;
        }
        return val.toFixed(decimals) + suffix;
    };
    const dataOrDefaultExp = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val) || val === Infinity) {
            return '0.00e+0' + suffix;
        }
        return val.toExponential(decimals) + suffix;
    };

    // --- CONSTANTES PHYSIQUES ET WGS84 ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
    const C_L = 299792458;      // Vitesse de la lumière (m/s)
    const G_U = 6.67430e-11;    // Constante gravitationnelle (N·m²/kg²)
    const WGS84_A = 6378137.0;  // Rayon équatorial
    const WGS84_E2 = 0.00669437999013; // Excentricité au carré
    const KMH_MS = 3.6;         // Conversion m/s vers km/h
    const R_AIR = 287.058;      // Constante spécifique de l'air sec
    const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
    const TEMP_SEA_LEVEL_K = 288.15; // Température ISA (Kelvin)
    const KELVIN_OFFSET = 273.15;
    const GPS_OPTS = { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 };
    const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
    const SOLAR_FLUX_DENSITY = 1361; // Irradiance solaire (W/m²)

    // --- PARAMÈTRES ET INTERVALLES ---
    const UKF_R_MAX = 500.0;     
    const IMU_UPDATE_RATE_MS = 20; // 50 Hz pour la fusion UKF
    const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz pour l'Astro/Météo
    const WEATHER_FETCH_INTERVAL_MS = 300000; // 5 minutes pour l'actualisation météo
    const SPEED_MOVING_THRESHOLD = 0.1; // m/s

    const ENVIRONMENT_FACTORS = {
        'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
        'FOREST': { MULT: 2.5, DISPLAY: 'Forêt' },
        'CONCRETE': { MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
        'METAL': { MULT: 5.0, DISPLAY: 'Métal/Bâtiment' }
    };
    
    // --- VARIABLES D'ÉTAT (Globales) ---
    let wID = null, domFastID = null, domSlowID = null;
    let lat = 43.2964, lon = 5.3697, kAlt = 0;
    let kSpd = 0.0, kUncert = UKF_R_MAX, kAltUncert = 10;
    let maxSpd = 0;
    let lastGPSPos = null; 
    let accel = { x: 0, y: 0, z: 0 }; 
    let gyro = { x: 0, y: 0, z: 0 };  
    let mag = { x: 0, y: 0, z: 0 };
    let light = null; 
    let sensorsStarted = false;
    let totalDistance = 0, timeMoving = 0, timeTotal = 0, startTime = null;
    let currentMass = 70.0;
    let currentCdA = 0.5;
    let lastP_hPa = 1013.25; 
    let lastT_K = TEMP_SEA_LEVEL_K;
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 340.29; 
    let local_g = 9.80665; 
    let emergencyStopActive = false;
    let currentCelestialBody = 'EARTH';
    let selectedEnvironment = 'NORMAL';
    let currentUKFReactivity = 'AUTO';
    let distanceRatioMode = false;
    let sunAltitudeRad = 0;

    // --- CLASSE UKF PROFESSIONNELLE (Architecture 21 États - Simplifiée) ---
    class ProfessionalUKF {
        constructor() {
            this.N_STATES = 21; 
            this.x = math.zeros(this.N_STATES); 
            this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
        }
        predict(imuData, dt) {
             // Simplifié : Mise à jour de la vitesse et de la position 
            const K_imu = 0.01;
            const a_long = imuData.accel[0]; // Accélération le long de l'axe (avant/arrière)
            this.x.set([3], this.x.get([3]) + a_long * dt * (1 - K_imu)); 
            this.x.set([0], this.x.get([0]) + (this.x.get([3]) / WGS84_A) * dt);
        }
        update(gpsData, R_dyn) {
            // Mise à jour simplifiée : fusion par filtre doux
            const K = 0.1; 
            if (gpsData.latitude) this.x.set([0], this.x.get([0]) * (1 - K) + gpsData.latitude * D2R * K);
            if (gpsData.longitude) this.x.set([1], this.x.get([1]) * (1 - K) + gpsData.longitude * D2R * K);
            if (gpsData.altitude !== null) this.x.set([2], this.x.get([2]) * (1 - K) + gpsData.altitude * K);
            if (gpsData.speed !== null) { this.x.set([3], this.x.get([3]) * (1 - K) + gpsData.speed * K); }
        }
        getState() {
            const x_data = this.x.toArray(); 
            return {
                lat: x_data[0] * R2D, lon: x_data[1] * R2D, alt: x_data[2],
                speed: x_data[3], 
                kUncert: this.P.get([3, 3]),
                kAltUncert: this.P.get([2, 2])
            };
        }
        }
    // =================================================================
// BLOC 2/5 : Métrologie, Physique Avancée & Outils Mathématiques
// =================================================================

    // --- FONCTIONS WGS84 et ATMOSPHÈRE ---
    function getWGS84Gravity(latitude, altitude) {
        const latRad = latitude * D2R; 
        const sin2lat = Math.sin(latRad) ** 2;
        // Gravité standard à la surface (Formule de Somigliana simplifiée)
        const g_surface = 9.780327 * (1 + 0.0053024 * sin2lat);
        // Correction d'altitude (Air Libre)
        return g_surface * (1 - 2 * altitude / WGS84_A); 
    }

    function getSpeedOfSound(tempK) {
        const gamma = 1.4; // Ratio des chaleurs spécifiques
        return Math.sqrt(gamma * R_AIR * (tempK || TEMP_SEA_LEVEL_K));
    }
    
    function calculateAirDensity(P_hPa, T_K) {
        // Formule des gaz parfaits : ρ = P / (R * T)
        return (P_hPa * 100) / (R_AIR * (T_K || TEMP_SEA_LEVEL_K)); 
    }

    function calculateDistanceRatio(kAlt) {
        // Simplification du mode "Nether" (Minecraft)
        return Math.max(1.0, 1.0 + (kAlt / 1000));
    }

    function dist3D(lat1, lon1, alt1, lat2, lon2, alt2) {
        const from = turf.point([lon1, lat1, alt1 || 0]); 
        const to = turf.point([lon2, lat2, alt2 || 0]);
        return turf.distance(from, to, { units: 'meters' }) || 0;
    }
    
    // --- CALCULS PHYSIQUES AVANCÉS (Relativité, Dynamique) ---
    function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, lat) {
        const V = kSpd || 0;
        
        // 1. Mise à jour de l'environnement
        currentAirDensity = calculateAirDensity(lastP_hPa, lastT_K);
        currentSpeedOfSound = getSpeedOfSound(lastT_K);
        local_g = getWGS84Gravity(lat, kAlt);
        
        // 2. Relativité
        const V_norm = V / C_L;
        const lorentzFactor = 1 / Math.sqrt(Math.max(0, 1 - Math.pow(V_norm, 2)));
        const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
        const gravitationalDilation = (local_g * kAlt / (C_L * C_L)) * 86400 * 1e9;
        const Rs_object = (2 * G_U * mass) / (C_L * C_L);
        const energy_rest_mass = mass * C_L * C_L;
        const kinetic_energy = mass * C_L * C_L * (lorentzFactor - 1); // Énergie Cinétique Relativiste
        const momentum = mass * V * lorentzFactor;
        
        // 3. Aérodynamique
        const machNumber = (currentSpeedOfSound > 0) ? V / currentSpeedOfSound : 0;
        const dynamicPressure = 0.5 * currentAirDensity * V * V; 
        const dragForce = dynamicPressure * CdA; 
        const dragPower_kW = (dragForce * V) / 1000.0;

        // 4. Géophysique
        const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
        const radiationPressure = (SOLAR_FLUX_DENSITY / C_L) * Math.max(0, Math.sin(sunAltitudeRad)); 
        
        return { 
            lorentzFactor, timeDilationSpeed, gravitationalDilation, Rs_object, energy_rest_mass,
            kinetic_energy, momentum, machNumber, dynamicPressure, dragForce, dragPower_kW,
            coriolisForce, radiationPressure
        };
    }
    
    // --- GESTION DES CORPS CÉLESTES/STATION ---
    function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
        let G_ACC_NEW = 9.80665; // EARTH
        if (body === 'MOON') G_ACC_NEW = 1.62;
        if (body === 'MARS') G_ACC_NEW = 3.72;
        if (body === 'ROTATING') {
            const centripetal_accel = angularVelocity * angularVelocity * rotationRadius;
            G_ACC_NEW = centripetal_accel; // Simule une gravité effective (accélération)
        }
        local_g = G_ACC_NEW;
        $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
                                              }
    // =================================================================
// BLOC 3/5 : Gestion des Capteurs, Temps NTP & Météo/API
// =================================================================

    // --- GESTION IMU (Accéléromètre/Gyroscope/Magnétomètre/Lumière) ---
    async function startIMUSensors() {
        if (sensorsStarted) return;
        sensorsStarted = true; 
        try {
            if (typeof Accelerometer === 'undefined') throw new Error("API Accelerometer non supportée.");

            const accSensor = new Accelerometer({ frequency: 50 });
            accSensor.addEventListener('reading', () => { accel.x = accSensor.x; accel.y = accSensor.y; accel.z = accSensor.z; });
            accSensor.start(); 
            
            const gyroSensor = new Gyroscope({ frequency: 50 });
            gyroSensor.addEventListener('reading', () => { gyro.x = gyroSensor.x; gyro.y = gyroSensor.y; gyro.z = gyroSensor.z; });
            gyroSensor.start(); 

            if (typeof Magnetometer !== 'undefined') {
                const magSensor = new Magnetometer({ frequency: 10 });
                magSensor.addEventListener('reading', () => { mag.x = magSensor.x; mag.y = magSensor.y; mag.z = magSensor.z; });
                magSensor.start();
            }
            if (typeof AmbientLightSensor !== 'undefined') {
                const lightSensor = new AmbientLightSensor({ frequency: 1 });
                lightSensor.addEventListener('reading', () => { light = lightSensor.illuminance; });
                lightSensor.start();
            }

            $('imu-status').textContent = "Actif (Multi-Capteurs)";
        } catch (error) {
            let errMsg = (error.name === 'SecurityError' || error.name === 'NotAllowedError') ? "Permission Capteurs Refusée." : error.message;
            $('imu-status').textContent = `❌ ${errMsg}`;
        }
    }

    function stopIMUSensors() {
        sensorsStarted = false;
        $('imu-status').textContent = 'Inactif';
        // En réalité, les capteurs doivent être arrêtés via leur instance. Ici, on simule l'arrêt de la lecture.
    }

    function stopSensorListeners() {
         if (domFastID) clearInterval(domFastID);
         domFastID = null;
    }

    // --- GESTION GPS (Geolocation API) ---
    function startGPSAcquisition(mode = 'HIGH_FREQ') {
        if (wID !== null) return;
        if (!('geolocation' in navigator)) {
            $('gps-status-dr').textContent = 'GPS NON SUPPORTÉ';
            return;
        }

        const gpsOptions = (mode === 'HIGH_FREQ') ? GPS_OPTS : { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 };
        wID = navigator.geolocation.watchPosition(
            (position) => {
                const { latitude, longitude, altitude, speed, accuracy } = position.coords;
                lat = latitude; lon = longitude; 
                lastGPSPos = { latitude, longitude, altitude, speed, accuracy }; 
                $('gps-status-dr').textContent = 'Signal OK';
                if (!startTime) startTime = Date.now();
                $('gps-precision').textContent = `${dataOrDefault(accuracy, 2)} m`;
                $('speed-raw-ms').textContent = `${dataOrDefault(speed, 2)} m/s`;
            },
            (error) => {
                const msg = (error.code === 1) ? 'Permission Refusée' : (error.code === 3) ? 'Timeout' : 'Signal Perdu';
                $('gps-status-dr').textContent = `Erreur: ${msg}`;
            },
            gpsOptions
        );
        $('gps-status-dr').textContent = 'Acquisition en cours...';
    }

    function stopGPSAcquisition() {
        if (wID !== null) navigator.geolocation.clearWatch(wID);
        wID = null;
        $('gps-status-dr').textContent = 'Arrêt: PAUSE GPS';
    }

    // --- CONTRÔLE PRINCIPAL (Toggle) ---
    function toggleGPS() {
        if (emergencyStopActive) return;
        if (wID === null) {
            startIMUSensors(); 
            startGPSAcquisition($('freq-select').value);
            if (!domFastID) startFastLoop();
            $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS'; 
        } else {
            stopGPSAcquisition(); 
            stopSensorListeners(); 
            $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS'; 
        }
    }
    
    // --- FETCH MÉTÉO (Simulation/Hors Ligne) ---
    function fetchWeather() {
        // En mode "Hors Ligne", on utilise des valeurs ISA ou statiques
        const newTempC = 15.0 + Math.sin(Date.now() / 10000000) * 5; // Simulation petite variation
        lastT_K = newTempC + KELVIN_OFFSET; 
        lastP_hPa = 1013.25; 
        const humidity = 60;
        
        $('weather-status').textContent = 'Actif (Hors Ligne)';
        if ($('temp-air-2')) $('temp-air-2').textContent = `${dataOrDefault(newTempC, 1)} °C`;
        if ($('pressure-2')) $('pressure-2').textContent = `${dataOrDefault(lastP_hPa, 0)} hPa`;
        if ($('humidity-2')) $('humidity-2').textContent = `${humidity} %`;
        
        currentAirDensity = calculateAirDensity(lastP_hPa, lastT_K);
        currentSpeedOfSound = getSpeedOfSound(lastT_K);
        if ($('air-density')) $('air-density').textContent = `${dataOrDefault(currentAirDensity, 3)} kg/m³`;
    }

    // --- SYNCHRONISATION NTP (Simplifié pour le HTML) ---
    let lServH = null, lLocH = null;
    function syncH() {
        // Simule la synchronisation NTP. En mode hors ligne, on utilise l'heure locale.
        lServH = Date.now();
        lLocH = Date.now();
        if ($('local-time')) $('local-time').textContent = new Date().toLocaleTimeString('fr-FR');
    }
    const getCDate = (lServH, lLocH) => lServH ? new Date(Date.now() + (lServH - lLocH)) : new Date();
    // =================================================================
// BLOC 4/5 : Boucles de Temps (Fast/Slow) & Logique d'Affichage
// =================================================================
    
    /**
     * BOUCLE RAPIDE (50Hz - UKF/Fusion) : Mise à jour des données critiques.
     */
    function startFastLoop() {
        if (domFastID) return;
        let lastTimestamp = performance.now();
        let oldLat = lat, oldLon = lon, oldAlt = kAlt;

        domFastID = setInterval(() => {
            if (emergencyStopActive) return;

            const now = performance.now();
            const dt = (now - lastTimestamp) / 1000.0;
            lastTimestamp = now;

            // 1. FUSION UKF
            const imuData = { accel: [accel.x, accel.y, accel.z], gyro: [gyro.x, gyro.y, gyro.z], dt };
            ukf.predict(imuData, dt); 
            if (lastGPSPos) {
                 const R_override = parseFloat($('gps-accuracy-override').value) || lastGPSPos.accuracy || UKF_R_MAX;
                 const R_mult = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
                 ukf.update(lastGPSPos, R_override * R_mult);
            }

            // 2. MISE À JOUR DE L'ÉTAT FILTRÉ & DISTANCE
            const estimatedState = ukf.getState();
            oldLat = lat; oldLon = lon; oldAlt = kAlt;
            lat = estimatedState.lat; lon = estimatedState.lon; kAlt = estimatedState.alt; 
            kSpd = estimatedState.speed; 
            kUncert = estimatedState.kUncert; 
            kAltUncert = estimatedState.kAltUncert;

            // Calcul du ratio de distance (Minecraft)
            const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt) : 1.0;
            if (oldLat !== null && oldLon !== null && lat !== null) {
                totalDistance += dist3D(oldLat, oldLon, oldAlt, lat, lon, kAlt) * ratio;
            }

            if (kSpd > maxSpd) maxSpd = kSpd;
            if (kSpd > SPEED_MOVING_THRESHOLD) timeMoving += dt;
            if (startTime) timeTotal = (Date.now() - startTime) / 1000;
            
            // 3. CALCULS PHYSIQUES AVANCÉS
            const physics = calculateAdvancedPhysics(kSpd, kAlt, currentMass, currentCdA, lat);
            
            // --- Mise à jour de l'affichage de la Vitesse et Relativité (Colonne 2) ---
            $('speed-stable').textContent = `${dataOrDefault(kSpd * KMH_MS, 1)} km/h`;
            $('speed-stable-ms').textContent = `${dataOrDefault(kSpd, 2)} m/s`;
            $('speed-stable-kms').textContent = `${dataOrDefault(kSpd / 1000, 4)} km/s`;
            $('speed-max').textContent = `${dataOrDefault(maxSpd * KMH_MS, 1)} km/h`;
            $('mach-number').textContent = dataOrDefault(physics.machNumber, 4);
            $('perc-speed-c').textContent = dataOrDefaultExp(kSpd / C_L * 100, 2, ' %');
            $('lorentz-factor').textContent = dataOrDefault(physics.lorentzFactor, 8);
            $('time-dilation-v').textContent = dataOrDefault(physics.timeDilationSpeed, 3, ' ns/j');
            $('time-dilation-g').textContent = dataOrDefault(physics.gravitationalDilation, 3, ' ns/j');
            $('Rs-object').textContent = dataOrDefaultExp(physics.Rs_object, 2, ' m');
            $('energy-rest-mass').textContent = dataOrDefaultExp(physics.energy_rest_mass, 2, ' J');
            $('momentum').textContent = dataOrDefaultExp(physics.momentum, 2, ' kg·m/s');

            // --- Mise à jour de la Dynamique (Colonne 3) ---
            $('gravity-local').textContent = `${dataOrDefault(local_g, 4)} m/s²`;
            $('accel-long').textContent = `${dataOrDefault(accel.x, 2)} m/s²`;
            $('force-g-long').textContent = `${dataOrDefault(accel.x / local_g, 2)} G`;
            $('angular-speed').textContent = `${dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2), 2)} rad/s`;
            $('dynamic-pressure').textContent = `${dataOrDefault(physics.dynamicPressure, 2)} Pa`;
            $('drag-force').textContent = `${dataOrDefault(physics.dragForce, 2)} N`;
            $('drag-power-kw').textContent = `${dataOrDefault(physics.dragPower_kW, 2)} kW`;
            $('kinetic-energy').textContent = `${dataOrDefaultExp(physics.kinetic_energy, 2)} J`;
            $('coriolis-force').textContent = `${dataOrDefault(physics.coriolisForce, 2)} N`;
            $('radiation-pressure').textContent = `${dataOrDefaultExp(physics.radiationPressure, 2)} N/m²`;

            // --- Mise à jour du Dashboard (Temps & Distance) ---
            $('elapsed-time').textContent = `${dataOrDefault(timeTotal, 2)} s`;
            $('time-moving').textContent = `${dataOrDefault(timeMoving, 2)} s`;
            $('distance-total-km').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km | ${dataOrDefault(totalDistance, 2)} m`;
            $('distance-ratio').textContent = dataOrDefault(ratio, 3);
            
            // --- Mise à jour EKF & IMU (Colonne 1 & 3) ---
            $('kalman-uncert').textContent = `${dataOrDefault(Math.sqrt(kUncert), 3)} m/s`;
            $('alt-uncertainty').textContent = `${dataOrDefault(Math.sqrt(kAltUncert), 2)} m`;
            $('gps-accuracy-display').textContent = `${dataOrDefault(parseFloat($('gps-accuracy-override').value), 2)} m (Forcé)`;
            $('accel-x').textContent = `${dataOrDefault(accel.x, 3)} m/s²`;
            $('accel-y').textContent = `${dataOrDefault(accel.y, 3)} m/s²`;
            $('accel-z').textContent = `${dataOrDefault(accel.z, 3)} m/s²`;
            $('mag-x').textContent = `${dataOrDefault(mag.x, 3)} μT`;
            $('mag-y').textContent = `${dataOrDefault(mag.y, 3)} μT`;
            $('mag-z').textContent = `${dataOrDefault(mag.z, 3)} μT`;
            $('ambient-light').textContent = `${dataOrDefault(light, 1)} lux`;

            // --- Mise à jour de la Carte (Colonne 2) ---
            if (lat !== null && lon !== null) {
                const newLatLng = [lat, lon];
                marker.setLatLng(newLatLng);
                circle.setLatLng(newLatLng).setRadius(Math.sqrt(kUncert));
                map.setView(newLatLng, map.getZoom()); 
                
                $('lat-display').textContent = lat.toFixed(5);
                $('lon-display').textContent = lon.toFixed(5);
                $('alt-display').textContent = `${dataOrDefault(kAlt, 1)} m`;
                $('heading-display').textContent = `${dataOrDefault(lastGPSPos?.heading, 1)} °`;
            }
        }, IMU_UPDATE_RATE_MS);
    }

    /**
     * BOUCLE LENTE (1Hz) : Horloge, Astro, Météo
     */
    function startSlowLoop() {
        if (domSlowID) return;
        domSlowID = setInterval(() => {
            const now = getCDate(lServH, lLocH); // Utilise l'heure synchronisée
            $('date-display').textContent = now.toLocaleDateString('fr-FR') + ' ' + now.toLocaleTimeString('fr-FR');
            
            if (lat !== null && lon !== null) {
                
                // Météo (Se déclenche toutes les 5 min)
                if (Date.now() % WEATHER_FETCH_INTERVAL_MS < DOM_SLOW_UPDATE_MS) {
                     fetchWeather(); 
                }

                // Astro (Soleil & Lune)
                const sunPos = SunCalc.getPosition(now, lat, lon);
                const moonPos = SunCalc.getPosition(now, lat, lon, 'moon');
                const times = SunCalc.getTimes(now, lat, lon);
                const moonIllumination = SunCalc.getMoonIllumination(now);
                sunAltitudeRad = sunPos.altitude;

                $('sun-alt').textContent = `${dataOrDefault(sunPos.altitude * R2D, 2)}°`;
                $('sun-azimuth').textContent = `${dataOrDefault(sunPos.azimuth * R2D, 2)}°`;
                $('sunrise-times').textContent = times.sunrise ? times.sunrise.toLocaleTimeString() : 'N/A';
                $('sunset-times').textContent = times.sunset ? times.sunset.toLocaleTimeString() : 'N/A';
                $('moon-illuminated').textContent = `${dataOrDefault(moonIllumination.fraction * 100, 1)} %`;
                $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
                $('moon-alt').textContent = `${dataOrDefault(moonPos.altitude * R2D, 2)}°`;
                $('moon-azimuth').textContent = `${dataOrDefault(moonPos.azimuth * R2D, 2)}°`;

                // Mise à jour de l'horloge céleste
                const sunAngle = (sunPos.altitude + (Math.PI / 2)) * R2D;
                const moonAngle = (moonPos.altitude + (Math.PI / 2)) * R2D;
                if ($('sun-element')) $('sun-element').style.transform = `rotate(${sunAngle}deg)`;
                if ($('moon-element')) $('moon-element').style.transform = `rotate(${moonAngle}deg)`;
            }
        }, DOM_SLOW_UPDATE_MS);
    }

    function getMoonPhaseName(phase) {
        if (phase === 0 || phase === 1) return "Nouvelle Lune";
        if (phase > 0 && phase < 0.25) return "Croissant Montant";
        if (phase === 0.25) return "Premier Quartier";
        if (phase > 0.25 && phase < 0.5) return "Gibbeuse Montante";
        if (phase === 0.5) return "Pleine Lune";
        if (phase > 0.5 && phase < 0.75) return "Gibbeuse Décroissante";
        if (phase === 0.75) return "Dernier Quartier";
        if (phase > 0.75 && phase < 1) return "Croissant Décroissant";
        return "N/A";
                                                                        }
    // =================================================================
// BLOC 5/5 : Initialisation (Map & Listeners)
// =================================================================
    
    // --- Initialisation de la Carte ---
    let map, marker, circle;
    function initMap() {
        const initialLatLng = [lat, lon];
        map = L.map('map').setView(initialLatLng, 15); 
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap' }).addTo(map);
        marker = L.marker(initialLatLng).addTo(map).bindPopup("Position UKF Estimée").openPopup();
        circle = L.circle(initialLatLng, { radius: Math.sqrt(UKF_R_MAX), color: 'red', fillColor: '#ff0000', fillOpacity: 0.2 }).addTo(map);
    }

    // --- Arrêt d'Urgence ---
    function emergencyStop() {
        emergencyStopActive = true;
        stopGPSAcquisition();
        stopSensorListeners();
        stopIMUSensors();
        if (domSlowID) clearInterval(domSlowID);
        $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: ACTIF 🔴';
        $('emergency-stop-btn').classList.add('active');
        $('toggle-gps-btn').disabled = true;
    }
    
    // --- Démarrage Principal ---
    document.addEventListener('DOMContentLoaded', () => {

        // 1. Initialisation
        initMap(); 
        ukf = new ProfessionalUKF(); 
        syncH(); // Synchro NTP

        // 2. Initialisation des valeurs par défaut/hors ligne
        fetchWeather(); 
        updateCelestialBody(currentCelestialBody, kAlt, 100, 0.0);

        // 3. LIAISON DES ÉVÉNEMENTS (CRITIQUE)
        
        // Bouton MARCHE/PAUSE
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').addEventListener('click', toggleGPS);
        }

        // Arrêt d'urgence
        if ($('emergency-stop-btn')) {
            $('emergency-stop-btn').addEventListener('click', emergencyStop);
        }

        // Réinitialisation des statistiques
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { totalDistance = 0; timeMoving = 0; maxSpd = 0; });
        if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });

        // Configuration du Filtre UKF/Environnement
        $('environment-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
        });
        $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
        $('freq-select').addEventListener('change', (e) => {
             // Re-démarrage GPS avec nouvelle fréquence si déjà actif
             if(wID !== null) { stopGPSAcquisition(); startGPSAcquisition(e.target.value); }
        });

        // Configuration Physique
        $('mass-input').addEventListener('input', (e) => { currentMass = parseFloat(e.target.value) || 70.0; $('mass-display').textContent = `${dataOrDefault(currentMass, 3)} kg`; });
        
        // Corps Céleste / Gravité
        const updateGravityControls = () => {
            currentCelestialBody = $('celestial-body-select').value;
            const radius = parseFloat($('rotation-radius').value) || 100;
            const angularV = parseFloat($('angular-velocity').value) || 0.0;
            updateCelestialBody(currentCelestialBody, kAlt, radius, angularV);
        };
        $('celestial-body-select').addEventListener('change', updateGravityControls);
        $('rotation-radius').addEventListener('input', updateGravityControls);
        $('angular-velocity').addEventListener('input', updateGravityControls);
        
        // Rapport Distance (Nether)
        $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
        });
        
        // Mode Nuit/Jour
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });

        // 4. DÉMARRAGE DES BOUCLES
        startSlowLoop(); // La boucle lente démarre toujours
        
        // L'état initial est en PAUSE
        stopGPSAcquisition(); 
        stopSensorListeners(); 
        stopIMUSensors();
        
        console.log("Système GNSS Dashboard v1.1 initialisé. Cliquez sur '▶️ MARCHE GPS' pour démarrer l'acquisition et la fusion UKF.");
    });
})(window);
