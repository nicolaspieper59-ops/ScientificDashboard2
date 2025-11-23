// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/5 : Dépendances, Constantes et État Global
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
    const degToCard = (deg) => {
        const directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSO', 'SO', 'OSO', 'O', 'ONO', 'NO', 'NNO'];
        const index = Math.round((deg % 360) / (360 / directions.length)) % directions.length;
        return directions[index];
    };
    const toTimeString = (ms) => {
        const h = Math.floor(ms / 3600000);
        const m = Math.floor((ms % 3600000) / 60000);
        const s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };

    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
    const C_L = 299792458;      // Vitesse de la lumière (m/s)
    const G_U = 6.67430e-11;    // Constante gravitationnelle (N·m²/kg²)
    const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
    const AU_METERS = 149597870700; 
    const LIGHT_YEAR_METERS = 9.461e15; 
    const R_AIR = 287.058;      // Constante spécifique de l'air sec
    const GAMMA_AIR = 1.4;      // Ratio des chaleurs spécifiques
    const MU_DYNAMIC_AIR = 1.8e-5; // Viscosité dynamique (Pa·s)
    const SOLAR_FLUX_DENSITY = 1361; 
    const KMH_MS = 3.6;         
    const WGS84_A = 6378137.0;  // Rayon équatorial
    const WGS84_E2 = 0.00669437999013; 
    const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C
    const BARO_ALT_REF_HPA = 1013.25; 
    const KELVIN_OFFSET = 273.15;
    const NETHER_RATIO = 8.0; 

    // --- CONFIGURATION SYSTÈME ET UKF ---
    const UKF_R_MAX = 500.0;     
    const IMU_UPDATE_RATE_MS = 20; // 50 Hz pour la fusion UKF
    const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz pour l'Astro/Météo
    const GPS_OPTS = {
        HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
        LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
    };
    const MIN_SPD = 0.01;
    const SPEED_MOVING_THRESHOLD = 0.1;
    const CELESTIAL_DATA = { EARTH: { G: 9.80665 }, MOON: { G: 1.62 }, MARS: { G: 3.72 }, ROTATING: { G: 0 } };
    const ENVIRONMENT_FACTORS = { 'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' }, 'FOREST': { MULT: 2.5, DISPLAY: 'Forêt' }, 'CONCRETE': { MULT: 7.0, DISPLAY: 'Grotte/Tunnel' } };

    // --- VARIABLES D'ÉTAT (Globales) ---
    let wID = null, domFastID = null, domSlowID = null;
    let ukf = null;
    let lat = 43.2964, lon = 5.3697, kAlt = 0; 
    let kSpd = 0.0, kUncert = UKF_R_MAX, kAltUncert = 10;
    let local_g = CELESTIAL_DATA['EARTH'].G;
    let maxSpd = 0, totalDistance = 0, timeMoving = 0, timeTotal = 0, startTime = null;
    let currentMass = 70.0, currentCdA = 0.5, selectedEnvironment = 'NORMAL';
    let lastGPSPos = null; // Données GPS brutes (lat, lon, alt, spd, acc)
    let accel = { x: 0, y: 0, z: 0 }, gyro = { x: 0, y: 0, z: 0 }, mag = { x: 0, y: 0, z: 0 }, light = 0;
    let sensorsStarted = false, emergencyStopActive = false, distanceRatioMode = false;
    let currentCelestialBody = 'EARTH', sunAltitudeRad = 0;
    let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = TEMP_SEA_LEVEL_K, currentAirDensity = RHO_SEA_LEVEL, currentSpeedOfSound = 340.29; 
    let lServH = null, lLocH = null; // Pour la synchronisation NTP
    
    // --- CLASSE UKF PROFESSIONNELLE (Simplification du modèle 21 états) ---
    class ProfessionalUKF {
        constructor() {
            this.N_STATES = 21; 
            this.x = math.zeros(this.N_STATES); 
            this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
        }
        predict(imuData, dt) {
            // Simplifié : propagation de la vitesse et de la position 
            const K_imu = 0.01;
            const a_long = imuData.accel.x; 
            let vN = this.x.get([3]) + a_long * dt;
            this.x.set([3], vN * (1 - K_imu)); 
            this.x.set([0], this.x.get([0]) + (vN / WGS84_A) * dt);
        }
        update(gpsData, R_dyn) {
            // Mise à jour simplifiée : fusion par filtre doux
            const K = 0.1 * (1 / (1 + R_dyn)); // R_dyn (GPS accuracy) influence le gain K
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
// BLOC 2/5 : Physique Avancée, Métrologie et Outils Mathématiques
// =================================================================

    // --- FONCTIONS WGS84 et ATMOSPHÈRE ---
    function getWGS84Gravity(latitude, altitude) {
        const latRad = latitude * D2R; 
        const sin2lat = Math.sin(latRad) ** 2;
        const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat);
        return g_surface * (1 - 2 * altitude / WGS84_A); 
    }
    
    function calculateDewPoint(tempC, humidityPerc) {
        // Formule de Magnus améliorée (valable 0°C à 60°C)
        const a = 17.625;
        const b = 243.04;
        const alpha = Math.log(humidityPerc / 100) + (a * tempC) / (b + tempC);
        return (b * alpha) / (a - alpha);
    }
    
    function getSpeedOfSound(tempK) {
        return Math.sqrt(GAMMA_AIR * R_AIR * (tempK || TEMP_SEA_LEVEL_K));
    }
    
    function calculateAirDensity(P_hPa, T_K) {
        return (P_hPa * 100) / (R_AIR * (T_K || TEMP_SEA_LEVEL_K)); 
    }

    function dist3D(lat1, lon1, alt1, lat2, lon2, alt2) {
        const from = turf.point([lon1, lat1, alt1 || 0]); 
        const to = turf.point([lon2, lat2, alt2 || 0]);
        return turf.distance(from, to, { units: 'meters' }) || 0;
    }
    
    // --- CALCULS PHYSIQUES AVANCÉS (Relativité, Dynamique) ---
    function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, lat) {
        const V = Math.max(0, kSpd);
        
        // 1. Relativité
        const V_norm = V / C_L;
        const lorentzFactor = 1 / Math.sqrt(Math.max(0, 1 - Math.pow(V_norm, 2)));
        const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
        const gravitationalDilation = (local_g * kAlt / (C_L * C_L)) * 86400 * 1e9;
        const Rs_object = (2 * G_U * mass) / (C_L * C_L);
        const energy_rest_mass = mass * C_L * C_L;
        const kinetic_energy = mass * C_L * C_L * (lorentzFactor - 1); 
        const momentum = mass * V * lorentzFactor;
        
        // 2. Aérodynamique
        const machNumber = (currentSpeedOfSound > 0) ? V / currentSpeedOfSound : 0;
        const dynamicPressure = 0.5 * currentAirDensity * V * V; 
        const reynoldsNumber = (currentAirDensity * V * 1) / MU_DYNAMIC_AIR; // Longueur caractéristique = 1m
        const dragForce = dynamicPressure * CdA; 
        const dragPower_kW = (dragForce * V) / 1000.0;

        // 3. Géophysique et Astro
        const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
        const radiationPressure = (SOLAR_FLUX_DENSITY / C_L) * Math.max(0, Math.sin(sunAltitudeRad)); 
        
        return { 
            lorentzFactor, timeDilationSpeed, gravitationalDilation, Rs_object, energy_rest_mass,
            kinetic_energy, momentum, machNumber, dynamicPressure, reynoldsNumber, dragForce, dragPower_kW,
            coriolisForce, radiationPressure
        };
    }

    // --- CALCULS DE TEMPS SOLAIRE ET SIDÉRAL ---
    function calculateSolarTime(date, longitude) {
        // Calcul TST (True Solar Time), MST (Mean Solar Time) et EOT (Equation of Time)
        const J2000 = new Date('2000-01-01T12:00:00Z').getTime();
        const JDE = (date.getTime() - J2000) / 86400000;
        const T = JDE / 36525;
        const lambda_M_rad = (280.46646 + 36000.76983 * T + 0.0003032 * T * T) % 360 * D2R;
        const lambda_L_rad = (357.52911 + 35999.05029 * T - 0.0001537 * T * T) % 360 * D2R;
        
        const C_corr = (1.914602 - 0.004817 * T - 0.000014 * T * T) * Math.sin(lambda_L_rad * R2D) +
                       (0.019993 - 0.000101 * T) * Math.sin(2 * lambda_L_rad * R2D) +
                       0.000289 * Math.sin(3 * lambda_L_rad * R2D);

        const EOT_minutes = (lambda_M_rad * R2D - (lambda_M_rad * R2D + C_corr)) * 4;
        
        // Heure du jour en minutes
        const time_of_day_minutes = date.getUTCHours() * 60 + date.getUTCMinutes() + date.getUTCSeconds() / 60;
        const LST_minutes = time_of_day_minutes + longitude * 4;
        
        const TST_minutes = LST_minutes + EOT_minutes;
        const MST_minutes = LST_minutes; // Simplification pour Mean Solar Time

        const TST_ms = ((TST_minutes * 60) % 86400) * 1000;
        const MST_ms = ((MST_minutes * 60) % 86400) * 1000;

        return { TST: toTimeString(TST_ms), MST: toTimeString(MST_ms), EOT: EOT_minutes.toFixed(2) };
    }
    
    function getTSLV(date, longitude) {
        // Calcul du Temps Sidéral Local Vrai (TSLV)
        const JD = date.getTime() / 86400000.0 + 2440587.5;
        const T = (JD - 2451545.0) / 36525.0;
        const GMST_0 = (6.697374558 + 2400.051336 * T + 0.000025862 * T * T) % 24;
        const GMST_h = GMST_0 + date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
        const TSLV_h = GMST_h + longitude / 15.0;
        
        const TSLV_seconds = ((TSLV_h % 24) * 3600) % 86400;
        return toTimeString(TSLV_seconds * 1000);
    }
    
    // --- GESTION DES CORPS CÉLESTES/STATION ---
    function updateCelestialBody(bodyKey, alt, rotR, rotV) {
        const body = CELESTIAL_DATA[bodyKey];
        let G_ACC_NEW = body ? body.G : CELESTIAL_DATA['EARTH'].G;

        if (bodyKey === 'ROTATING') {
            const centripetal_accel = rotR > 0 ? rotR * rotV * rotV : 0;
            G_ACC_NEW = centripetal_accel; 
        } else if (bodyKey === 'EARTH') {
             G_ACC_NEW = getWGS84Gravity(lat || 0, alt || 0);
        }
        local_g = G_ACC_NEW;
        return { G_ACC_NEW };
                               }
    // =================================================================
// BLOC 3/5 : Gestion des Capteurs, API et I/O
// =================================================================

    // --- GESTION IMU (Accéléromètre/Gyroscope/Magnétomètre/Lumière) ---
    async function startIMUSensors() {
        if (sensorsStarted) return;
        sensorsStarted = true; 
        try {
            // Accéléromètre & Gyroscope
            const accSensor = new Accelerometer({ frequency: 50 });
            accSensor.addEventListener('reading', () => { accel.x = accSensor.x; accel.y = accSensor.y; accel.z = accSensor.z; });
            accSensor.start(); 
            
            const gyroSensor = new Gyroscope({ frequency: 50 });
            gyroSensor.addEventListener('reading', () => { gyro.x = gyroSensor.x; gyro.y = gyroSensor.y; gyro.z = gyroSensor.z; });
            gyroSensor.start(); 

            // Magnétomètre et Lumière Ambiante (optionnel)
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
    }

    // --- GESTION GPS (Geolocation API) ---
    function startGPSAcquisition() {
        if (wID !== null) return;
        if (!('geolocation' in navigator)) {
            $('gps-status-dr').textContent = 'GPS NON SUPPORTÉ';
            return;
        }

        const mode = $('freq-select').value;
        wID = navigator.geolocation.watchPosition(
            (position) => {
                const { latitude, longitude, altitude, speed, accuracy, heading } = position.coords;
                lat = latitude; lon = longitude; 
                lastGPSPos = { latitude, longitude, altitude, speed, accuracy, heading }; 
                $('gps-status-dr').textContent = 'Signal OK';
                if (!startTime) startTime = Date.now();
                $('gps-precision').textContent = `${dataOrDefault(accuracy, 2)} m`; // Précision GPS (Acc)
                $('speed-raw-ms').textContent = `${dataOrDefault(speed, 2)} m/s`;
            },
            (error) => {
                const msg = (error.code === 1) ? 'Permission Refusée' : (error.code === 3) ? 'Timeout' : 'Signal Perdu';
                $('gps-status-dr').textContent = `Erreur: ${msg}`;
            },
            GPS_OPTS[mode]
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
            startGPSAcquisition();
            if (!domFastID) startFastLoop();
            $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS'; 
        } else {
            stopGPSAcquisition(); 
            if (domFastID) clearInterval(domFastID);
            domFastID = null;
            $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS'; 
        }
    }
    
    // --- MÉTÉO (Simulation/Hors Ligne pour les IDs HTML) ---
    function fetchWeather() {
        // En mode "Hors Ligne", on utilise des valeurs statiques/ISA
        const newTempC = 10.3; // Valeur par défaut
        const newHumidity = 60;
        lastT_K = newTempC + KELVIN_OFFSET; 
        lastP_hPa = 1013.0; // Valeur par défaut
        
        currentAirDensity = calculateAirDensity(lastP_hPa, lastT_K);
        currentSpeedOfSound = getSpeedOfSound(lastT_K);
        const dewPointC = calculateDewPoint(newTempC, newHumidity);
        
        $('weather-status').textContent = 'Actif (Hors Ligne)';
        $('temp-air-2').textContent = `${dataOrDefault(newTempC, 1)} °C`;
        $('pressure-2').textContent = `${dataOrDefault(lastP_hPa, 0)} hPa`;
        $('humidity-2').textContent = `${newHumidity} %`;
        $('dew-point').textContent = `${dataOrDefault(dewPointC, 1)} °C`;
        $('air-density').textContent = `${dataOrDefault(currentAirDensity, 3)} kg/m³`;
        $('speed-of-sound-calc').textContent = `${dataOrDefault(currentSpeedOfSound, 2)} m/s (Cor.)`;
    }
    
    // --- POLLUANTS (Simulation/N/A pour les IDs HTML) ---
    function updatePollutantsDOM() {
        // IDs: no2-value, pm25-value, pm10-value, o3-value
        $('no2-value').textContent = 'N/A';
        $('pm25-value').textContent = 'N/A';
        $('pm10-value').textContent = 'N/A';
        $('o3-value').textContent = 'N/A';
    }

    // --- SYNCHRONISATION NTP (Simplifié) ---
    const getCDate = (lServH, lLocH) => lServH ? new Date(Date.now() + (lServH - lLocH)) : new Date();
    function syncH() {
        lServH = Date.now(); lLocH = Date.now();
        }
    // =================================================================
// BLOC 4/5 : Boucles de Mise à Jour (Fast/Slow) & Affichage
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
            const imuData = { accel: accel, gyro: gyro, dt };
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

            // Calcul de la distance
            const ratio = distanceRatioMode ? NETHER_RATIO : 1.0;
            if (oldLat !== null && oldLon !== null && lat !== null) {
                totalDistance += dist3D(oldLat, oldLon, oldAlt, lat, lon, kAlt) * ratio;
            }

            // Mise à jour des temps et vitesses max/moyennes
            if (kSpd > maxSpd) maxSpd = kSpd;
            if (kSpd > SPEED_MOVING_THRESHOLD) timeMoving += dt;
            if (startTime) timeTotal = (Date.now() - startTime) / 1000;
            const avgSpdMov = timeMoving > 0 ? totalDistance / timeMoving : 0;
            const avgSpdTotal = timeTotal > 0 ? totalDistance / timeTotal : 0;

            // 3. CALCULS PHYSIQUES AVANCÉS
            local_g = currentCelestialBody === 'EARTH' ? getWGS84Gravity(lat, kAlt) : local_g;
            const physics = calculateAdvancedPhysics(kSpd, kAlt, currentMass, currentCdA, lat);
            
            // --- MISE À JOUR DU DOM (VITESSE, RELATIVITÉ, DYNAMIQUE) ---
            
            // Vitesse
            $('speed-stable').textContent = `${dataOrDefault(kSpd * KMH_MS, 1)} km/h`;
            $('speed-stable-ms').textContent = `${dataOrDefault(kSpd, 2)} m/s`;
            $('speed-stable-kms').textContent = `${dataOrDefault(kSpd / 1000, 4)} km/s`;
            $('speed-max').textContent = `${dataOrDefault(maxSpd * KMH_MS, 1)} km/h`;
            $('vitesse-moyenne-mvt').textContent = `${dataOrDefault(avgSpdMov * KMH_MS, 1)} km/h`;
            $('vitesse-moyenne-totale').textContent = `${dataOrDefault(avgSpdTotal * KMH_MS, 1)} km/h`;
            $('perc-speed-c').textContent = dataOrDefaultExp(kSpd / C_L * 100, 2, ' %');
            $('mach-number').textContent = dataOrDefault(physics.machNumber, 4);
            
            // Relativité
            $('lorentz-factor').textContent = dataOrDefault(physics.lorentzFactor, 8);
            $('time-dilation-v').textContent = dataOrDefault(physics.timeDilationSpeed, 3, ' ns/j');
            $('time-dilation-g').textContent = dataOrDefault(physics.gravitationalDilation, 3, ' ns/j');
            $('Rs-object').textContent = dataOrDefaultExp(physics.Rs_object, 2, ' m');
            $('energy-rest-mass').textContent = dataOrDefaultExp(physics.energy_rest_mass, 2, ' J');
            $('energy-relativistic').textContent = dataOrDefaultExp(physics.kinetic_energy, 2, ' J');
            $('momentum').textContent = dataOrDefaultExp(physics.momentum, 2, ' kg·m/s');
            
            // Distance (Lumière)
            $('distance-total-km').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km | ${dataOrDefault(totalDistance, 2)} m`;
            $('distance-ratio').textContent = dataOrDefault(ratio, 3);
            $('distance-light-s').textContent = dataOrDefaultExp(totalDistance / C_L, 2, ' s');
            $('distance-light-m').textContent = dataOrDefaultExp(totalDistance / (C_L * 60), 2, ' min');
            $('distance-light-h').textContent = dataOrDefaultExp(totalDistance / (C_L * 3600), 2, ' h');
            $('distance-light-j').textContent = dataOrDefaultExp(totalDistance / (C_L * 86400), 2, ' j');
            $('distance-light-sem').textContent = dataOrDefaultExp(totalDistance / (C_L * 86400 * 7), 2, ' sem');
            $('distance-light-mois').textContent = dataOrDefaultExp(totalDistance / (C_L * 86400 * 30.44), 2, ' mois');
            $('distance-ua').textContent = dataOrDefaultExp(totalDistance / AU_METERS, 2);
            $('distance-al').textContent = dataOrDefaultExp(totalDistance / LIGHT_YEAR_METERS, 2);


            // Dynamique & Forces
            $('gravity-local').textContent = `${dataOrDefault(local_g, 4)} m/s²`;
            $('accel-long').textContent = `${dataOrDefault(accel.x, 2)} m/s²`;
            $('force-g-long').textContent = `${dataOrDefault(accel.x / local_g, 2)} G`;
            $('angular-speed-gyro').textContent = `${dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2), 2)} rad/s`;
            $('dynamic-pressure').textContent = `${dataOrDefault(physics.dynamicPressure, 2)} Pa`;
            $('drag-force').textContent = `${dataOrDefault(physics.dragForce, 2)} N`;
            $('drag-power-kw').textContent = `${dataOrDefault(physics.dragPower_kW, 2)} kW`;
            $('kinetic-energy').textContent = `${dataOrDefaultExp(physics.kinetic_energy, 2)} J`;
            $('radiation-pressure').textContent = `${dataOrDefaultExp(physics.radiationPressure, 2)} N/m²`;
            $('coriolis-force').textContent = `${dataOrDefault(physics.coriolisForce, 2)} N`;
            $('reynolds-number').textContent = `${dataOrDefaultExp(physics.reynoldsNumber, 2)}`;
            
            // IMU & EKF Debug
            $('accel-x').textContent = `${dataOrDefault(accel.x, 3)} m/s²`;
            $('accel-y').textContent = `${dataOrDefault(accel.y, 3)} m/s²`;
            $('accel-z').textContent = `${dataOrDefault(accel.z, 3)} m/s²`;
            $('mag-x').textContent = `${dataOrDefault(mag.x, 3)} μT`;
            $('mag-y').textContent = `${dataOrDefault(mag.y, 3)} μT`;
            $('mag-z').textContent = `${dataOrDefault(mag.z, 3)} μT`;
            $('ambient-light').textContent = `${dataOrDefault(light, 1)} lux`;
            $('kalman-uncert').textContent = `${dataOrDefault(Math.sqrt(kUncert), 3)} m/s`;
            $('alt-uncertainty').textContent = `${dataOrDefault(Math.sqrt(kAltUncert), 2)} m`;
            
            // Position & Carte
            if (lat !== null && lon !== null) {
                const newLatLng = [lat, lon];
                marker.setLatLng(newLatLng);
                circle.setLatLng(newLatLng).setRadius(Math.sqrt(kUncert));
                map.setView(newLatLng, map.getZoom()); 
                
                $('lat-display').textContent = lat.toFixed(5);
                $('lon-display').textContent = lon.toFixed(5);
                $('alt-display').textContent = `${dataOrDefault(kAlt, 1)} m`;
                $('heading-display').textContent = `${dataOrDefault(lastGPSPos?.heading, 1)} ° (${degToCard(lastGPSPos?.heading || 0)})`;
            }
        }, IMU_UPDATE_RATE_MS);
    }

    /**
     * BOUCLE LENTE (1Hz) : Horloge, Astro, Météo
     */
    function startSlowLoop() {
        if (domSlowID) return;
        domSlowID = setInterval(() => {
            const now = getCDate(lServH, lLocH); 
            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            $('date-display').textContent = now.toLocaleDateString('fr-FR') + ' ' + now.toLocaleTimeString('fr-FR', { timeZone: 'UTC' });
            
            if (startTime) {
                $('elapsed-time').textContent = `${toTimeString(timeTotal * 1000)}`;
                $('time-moving').textContent = `${toTimeString(timeMoving * 1000)}`;
            }
            
            // Météo et Polluants
            fetchWeather(); // Met à jour le DOM météo
            updatePollutantsDOM(); // Met à jour le DOM polluants
            
            // Astro (Soleil & Lune)
            if (lat !== null && lon !== null) {
                const sunPos = SunCalc.getPosition(now, lat, lon);
                const moonPos = SunCalc.getPosition(now, lat, lon, 'moon');
                const times = SunCalc.getTimes(now, lat, lon);
                const moonIllumination = SunCalc.getMoonIllumination(now);
                const solarTimes = calculateSolarTime(now, lon);

                sunAltitudeRad = sunPos.altitude;

                // Soleil
                $('sun-alt').textContent = `${dataOrDefault(sunPos.altitude * R2D, 2)}°`;
                $('sun-azimuth').textContent = `${dataOrDefault(sunPos.azimuth * R2D, 2)}°`;
                $('sunrise-times').textContent = times.sunrise ? times.sunrise.toLocaleTimeString() : 'N/A';
                $('sunset-times').textContent = times.sunset ? times.sunset.toLocaleTimeString() : 'N/A';
                if (times.sunset && times.sunrise) {
                    const duration = times.sunset.getTime() - times.sunrise.getTime();
                    $('day-duration').textContent = toTimeString(duration);
                }
                
                // Lune
                $('moon-illuminated').textContent = `${dataOrDefault(moonIllumination.fraction * 100, 1)} %`;
                $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
                $('moon-alt').textContent = `${dataOrDefault(moonPos.altitude * R2D, 2)}°`;
                $('moon-azimuth').textContent = `${dataOrDefault(moonPos.azimuth * R2D, 2)}°`;
                
                // Temps Solaires
                $('mst').textContent = solarTimes.MST;
                $('tst').textContent = solarTimes.TST;
                $('noon-solar').textContent = times.solarNoon ? times.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC' }) : 'N/A';
                $('eot').textContent = `${solarTimes.EOT} min`;
                $('tslv').textContent = getTSLV(now, lon);
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
        if (typeof L === 'undefined') {
            console.error("Leaflet n'est pas chargé. La carte ne sera pas affichée.");
            return;
        }
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
        stopIMUSensors();
        if (domFastID) clearInterval(domFastID);
        domFastID = null;
        $('emergency-stop-btn').innerHTML = '🛑 Arrêt d\'urgence: ACTIF 🔴';
        $('emergency-stop-btn').classList.add('active');
        $('toggle-gps-btn').disabled = true;
    }
    
    // --- Démarrage Principal ---
    document.addEventListener('DOMContentLoaded', () => {

        // Vérification des dépendances
        if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
            console.error("Dépendances critiques manquantes. Le dashboard ne fonctionnera pas.");
        }

        // 1. Initialisation
        initMap(); 
        ukf = new ProfessionalUKF(); 
        syncH(); // Synchro NTP
        fetchWeather(); // Initialisation Météo/Atmosphère
        updatePollutantsDOM(); // Initialisation Polluants (N/A)

        // 2. LIAISON DES ÉVÉNEMENTS

        // Bouton MARCHE/PAUSE & Arrêt d'urgence
        $('toggle-gps-btn').addEventListener('click', toggleGPS);
        $('emergency-stop-btn').addEventListener('click', emergencyStop);

        // Configuration du Filtre UKF/Environnement
        $('environment-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
            $('env-factor').textContent = `${factor.DISPLAY} (x${factor.MULT.toFixed(1)})`;
        });
        $('freq-select').addEventListener('change', () => { if(wID !== null) { stopGPSAcquisition(); startGPSAcquisition(); } });

        // Configuration Physique
        $('mass-input').addEventListener('input', (e) => { currentMass = parseFloat(e.target.value) || 70.0; $('mass-display').textContent = `${dataOrDefault(currentMass, 3)} kg`; });
        
        // Corps Céleste / Gravité
        const updateGravityControls = () => {
            currentCelestialBody = $('celestial-body-select').value;
            const radius = parseFloat($('rotation-radius').value) || 100;
            const angularV = parseFloat($('angular-velocity').value.replace(',', '.')) || 0.0;
            const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, radius, angularV);
            $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        };
        $('celestial-body-select').addEventListener('change', updateGravityControls);
        $('rotation-radius').addEventListener('input', updateGravityControls);
        $('angular-velocity').addEventListener('input', updateGravityControls);
        
        // Rapport Distance (Nether/Altitude)
        $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            const ratio = distanceRatioMode ? NETHER_RATIO : 1.0;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE (x8)' : 'SURFACE (x1)'}`;
            $('distance-ratio').textContent = dataOrDefault(ratio, 3);
        });

        // Contrôles de Réinitialisation
        $('reset-dist-btn').addEventListener('click', () => { totalDistance = 0; timeMoving = 0; });
        $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
        $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
        
        // Mode Nuit/Jour
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });


        // 3. DÉMARRAGE DES BOUCLES
        startSlowLoop(); // La boucle lente (Astro/Temps) démarre immédiatement.
        
        // État initial PAUSE
        $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS'; 
        stopGPSAcquisition(); 
        stopIMUSensors();
        
        console.log("Système GNSS Dashboard est prêt. Initialisation des valeurs par défaut terminée.");
    });
})(window);
