// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// COUPE 1/5 : Dépendances, Constantes et État Global
// =================================================================
((window) => {

    // --- FONCTIONS UTILITAIRES GLOBALES ---
    const $ = id => document.getElementById(id);
    const dataOrDefault = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val)) {
            return (decimals === 0 ? '0' : '0.00') + suffix;
        }
        return val.toFixed(decimals) + suffix;
    };
    const dataOrDefaultExp = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val)) {
            const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
            return zeroDecimals + 'e+0' + suffix;
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

    // --- PARAMÈTRES ET INTERVALLES ---
    const UKF_R_MAX = 500.0;     
    const IMU_UPDATE_RATE_MS = 20; // 50 Hz pour la fusion UKF
    const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz pour l'Astro/Météo

    // --- VARIABLES D'ÉTAT (Globales) ---
    let wID = null, domFastID = null; // IDs pour les interval/watch
    let lat = 43.2964, lon = 5.3697, kAlt = 0; // Position EKF filtrée
    let kSpd = 0.0, kUncert = UKF_R_MAX; // Vitesse et Incertitude EKF
    let lastGPSPos = null; // Cache des dernières données GPS brutes
    let accel = { x: 0, y: 0, z: 0 }; // Données brutes Accéléromètre
    let gyro = { x: 0, y: 0, z: 0 };  // Données brutes Gyroscope
    let sensorsStarted = false; // Flag de l'état IMU
    let totalDistance = 0;
    let currentMass = 70.0;
    let currentCdA = 0.5;
    let lastP_hPa = 1013.25; 
    let lastT_K = TEMP_SEA_LEVEL_K;
    let currentSpeedOfSound = 340.29; 
    let local_g = 9.80665; 

    // --- CLASSE UKF PROFESSIONNELLE (Architecture 21 États - Simplifiée) ---
    // (L'implémentation complète du modèle 21 états est simplifiée ici pour la démo)
    class ProfessionalUKF {
        constructor() {
            this.N_STATES = 21; 
            this.x = math.zeros(this.N_STATES); 
            this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
        }
        predict(imuData, dt) { /* Logique de propagation UKF (utilise Accel/Gyro) */ }
        update(gpsData, R_dyn) {
            // Mise à jour simplifiée : fusion par moyenne pondérée (filtre doux)
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
                speed: x_data[3], // Simplification : Vitesse est directement l'état
                kUncert: this.P.get([3, 3]), 
            };
        }
            }
    // =================================================================
// COUPE 2/5 : Logique des Capteurs (GPS & IMU) et Contrôles
// =================================================================

    // --- GESTION IMU (Accéléromètre/Gyroscope) ---
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

            $('imu-status').textContent = "Actif (Multi-Capteurs)";
        } catch (error) {
            let errMsg = (error.name === 'SecurityError' || error.name === 'NotAllowedError') ? "Permission Capteurs Refusée." : error.message;
            $('imu-status').textContent = `❌ ${errMsg}`;
        }
    }

    function stopIMUSensors() {
        // En l'absence de références directes aux capteurs, on se contente d'arrêter la boucle UKF
        sensorsStarted = false;
        $('imu-status').textContent = 'Inactif';
    }

    function stopSensorListeners() {
         if (domFastID) clearInterval(domFastID);
         domFastID = null;
         // Réinitialisation des données brutes
         accel = { x: 0, y: 0, z: 0 };
         gyro = { x: 0, y: 0, z: 0 };
    }

    // --- GESTION GPS (Geolocation API) ---
    function startGPSAcquisition() {
        if (wID !== null) return;
        if (!('geolocation' in navigator)) {
            $('gps-status-line').textContent = 'GPS NON SUPPORTÉ';
            return;
        }

        wID = navigator.geolocation.watchPosition(
            (position) => {
                const { latitude, longitude, altitude, speed, accuracy } = position.coords;
                lat = latitude; lon = longitude; // Mise à jour immédiate pour l'UKF
                lastGPSPos = { latitude, longitude, altitude, speed, accuracy }; 
                $('gps-status-line').textContent = 'Signal OK';
            },
            (error) => {
                const msg = (error.code === 1) ? 'Permission Refusée' : (error.code === 3) ? 'Timeout' : 'Signal Perdu';
                $('gps-status-line').textContent = `Erreur: ${msg}`;
            },
            GPS_OPTS
        );
        $('gps-status-line').textContent = 'Acquisition en cours...';
    }

    function stopGPSAcquisition() {
        if (wID !== null) navigator.geolocation.clearWatch(wID);
        wID = null;
        $('gps-status-line').textContent = 'Arrêt: PAUSE GPS';
        $('gps-accuracy-display').textContent = 'N/A';
    }

    // --- CONTRÔLE PRINCIPAL (Le bouton) ---
    function toggleGPS() {
        if (wID === null) {
            // MARCHE
            startIMUSensors(); 
            startGPSAcquisition();
            if (!domFastID) startFastLoop(); // S'assure que la boucle UKF démarre
            $('gps-toggle-btn').innerHTML = '⏸️ PAUSE GPS'; 
        } else {
            // PAUSE
            stopGPSAcquisition(); 
            stopSensorListeners(); 
            stopIMUSensors();
            $('gps-toggle-btn').innerHTML = '▶️ MARCHE GPS'; 
        }
        }
    // =================================================================
// COUPE 3/5 : Physique, Métrologie et Modélisation
// =================================================================

    // --- FONCTIONS WGS84 et ATMOSPHÈRE ---
    function getWGS84Gravity(latitude, altitude) {
        const latRad = latitude * D2R; 
        const sin2lat = Math.sin(latRad) ** 2;
        // Gravité standard à la surface
        const g_surface = 9.780327 * (1 + 0.0053024 * sin2lat);
        // Correction d'altitude (Bouguer/Air Libre simplifiée)
        return g_surface * (1 - 2 * altitude / WGS84_A); 
    }

    function getSpeedOfSound(tempK) {
        if(tempK < KELVIN_OFFSET) tempK = TEMP_SEA_LEVEL_K;
        const R_specific = 287.058; // Constante de l'air
        const gamma = 1.4;          // Ratio des chaleurs spécifiques
        return Math.sqrt(gamma * R_specific * tempK);
    }
    
    function calculateAirDensity(P_hPa, T_K) {
        return (P_hPa * 100) / (R_AIR * T_K); // Pression en Pa
    }

    function dist3D(lat1, lon1, alt1, lat2, lon2, alt2) {
        const from = turf.point([lon1, lat1, alt1 || 0]); 
        const to = turf.point([lon2, lat2, alt2 || 0]);
        return turf.distance(from, to, { units: 'meters' });
    }

    // --- CALCULS PHYSIQUES AVANCÉS (Relativité, Dynamique) ---
    function calculateAdvancedPhysics(kSpd, currentMass, currentCdA) {
        const V = kSpd || 0;
        const airDensity = calculateAirDensity(lastP_hPa, lastT_K);
        currentSpeedOfSound = getSpeedOfSound(lastT_K);

        const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(V / C_L, 2));
        const kinetic_energy = 0.5 * currentMass * Math.pow(V, 2);
        const momentum = currentMass * V * lorentzFactor; 
        
        const machNumber = (currentSpeedOfSound > 0) ? V / currentSpeedOfSound : 0;
        const dynamicPressure = 0.5 * airDensity * V * V; 
        const dragForce = dynamicPressure * currentCdA; // Force de Traînée
        const dragPower_kW = (dragForce * V) / 1000.0;

        return { 
            lorentzFactor, kinetic_energy, momentum,
            machNumber, dynamicPressure, dragForce, dragPower_kW
        };
    }

    // --- MISE À JOUR MÉTÉO SIMULÉE (Remplacer par API) ---
    function fetchWeather(lat, lon) {
        // En mode "Hors Ligne", on utilise un modèle ISA corrigé ou des valeurs statiques
        lastT_K = 293.15; // 20°C
        lastP_hPa = 1014.2; 
        
        $('weather-status').textContent = 'Actif (Hors Ligne)';
        $('temp-air-2').textContent = `${(lastT_K - KELVIN_OFFSET).toFixed(1)} °C`;
        $('pressure-2').textContent = `${lastP_hPa.toFixed(0)} hPa`;
        $('humidity-2').textContent = '60 %'; // Valeur par défaut
            }
    // =================================================================
// COUPE 4/5 : Boucles de Mise à Jour (UKF & Astro)
// =================================================================

    /**
     * BOUCLE RAPIDE (50Hz - UKF/Fusion) : Mise à jour des données critiques.
     */
    function startFastLoop() {
        if (domFastID) return;
        let lastTimestamp = performance.now();
        let oldLat = lat, oldLon = lon, oldAlt = kAlt;

        domFastID = setInterval(() => {
            const now = performance.now();
            const dt = (now - lastTimestamp) / 1000.0;
            lastTimestamp = now;

            // 1. FUSION UKF
            const imuData = { accel: [accel.x, accel.y, accel.z], gyro: [gyro.x, gyro.y, gyro.z], dt };
            ukf.predict(imuData, dt); 
            if (lastGPSPos) {
                 ukf.update(lastGPSPos, lastGPSPos.accuracy || UKF_R_MAX);
            }

            // 2. MISE À JOUR DE L'ÉTAT FILTRÉ & DISTANCE
            const estimatedState = ukf.getState();
            oldLat = lat; oldLon = lon; oldAlt = kAlt;
            lat = estimatedState.lat; lon = estimatedState.lon; kAlt = estimatedState.alt; 
            kSpd = estimatedState.speed; kUncert = estimatedState.kUncert; 
            
            if (oldLat !== null && oldLon !== null && lat !== null) {
                totalDistance += dist3D(oldLat, oldLon, oldAlt, lat, lon, kAlt);
            }

            // 3. CALCULS PHYSIQUES ET DYNAMIQUE
            local_g = getWGS84Gravity(lat, kAlt);
            const physics = calculateAdvancedPhysics(kSpd, currentMass, currentCdA);
            
            // --- Mise à jour du DOM ---
            $('speed-kmh').textContent = `${dataOrDefault(kSpd * KMH_MS, 1)} km/h`;
            $('stable-speed-ms').textContent = `${dataOrDefault(kSpd, 2)} m/s`;
            $('mach-number').textContent = dataOrDefault(physics.machNumber, 4);
            $('lorentz-factor').textContent = dataOrDefault(physics.lorentzFactor, 8);
            $('dynamic-pressure').textContent = `${dataOrDefault(physics.dynamicPressure, 2)} Pa`;
            $('drag-force').textContent = `${dataOrDefault(physics.dragForce, 2)} N`;
            $('total-distance').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km`;
            
            // --- Carte et Position ---
            if (lat !== null && lon !== null) {
                marker.setLatLng([lat, lon]);
                circle.setLatLng([lat, lon]).setRadius(Math.sqrt(kUncert));
                map.setView([lat, lon], map.getZoom()); 
                $('lat-ekf').textContent = lat.toFixed(5);
                $('lon-ekf').textContent = lon.toFixed(5);
                $('alt-ekf').textContent = `${dataOrDefault(kAlt, 1)} m`;
            }
        }, IMU_UPDATE_RATE_MS);
    }

    /**
     * BOUCLE LENTE (1Hz) : Horloge, Astro, Météo
     */
    function startSlowLoop() {
        if (domSlowID) return;
        domSlowID = setInterval(() => {
            const now = new Date();
            $('date-time-utc').textContent = now.toUTCString();
            
            // Météo et Correction atmosphérique
            if (wID !== null && lat !== null) {
                fetchWeather(lat, lon);
                $('speed-of-sound').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Cor.)`;
            }
            
            // Astro (Utilise SunCalc)
            if (lat !== null && lon !== null) {
                const sunPos = SunCalc.getPosition(now, lat, lon);
                const times = SunCalc.getTimes(now, lat, lon);
                
                $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
                $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)}°`;
                $('sunrise-times').textContent = times.sunrise ? times.sunrise.toLocaleTimeString() : 'N/A';
                $('sunset-times').textContent = times.sunset ? times.sunset.toLocaleTimeString() : 'N/A';
            }
        }, DOM_SLOW_UPDATE_MS);
        }
    // =================================================================
// COUPE 5/5 : Initialisation et Démarrage
// =================================================================
    
    // --- Initialisation de la Carte (Doit être appelée après le chargement du DOM) ---
    let map, marker, circle;
    function initMap() {
        const initialLatLng = [lat, lon];
        map = L.map('map').setView(initialLatLng, 15); 
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap' }).addTo(map);
        marker = L.marker(initialLatLng).addTo(map).bindPopup("Position UKF Estimée").openPopup();
        circle = L.circle(initialLatLng, { radius: Math.sqrt(UKF_R_MAX), color: '#007bff', fillColor: '#007bff', fillOpacity: 0.2 }).addTo(map);
    }

    // --- Démarrage Principal ---
    document.addEventListener('DOMContentLoaded', () => {

        // Vérification finale des dépendances (Redondant, mais sécuritaire)
        if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
            return; // Arrêt du script si la coupe 1 a échoué
        }

        initMap(); // Initialise la carte
        ukf = new ProfessionalUKF(); // Initialise le filtre

        // Lier le bouton MARCHE/PAUSE (CRITIQUE pour le fonctionnement)
        if ($('gps-toggle-btn')) {
            $('gps-toggle-btn').addEventListener('click', toggleGPS);
        }

        // --- Configuration initiale du Dashboard ---
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        $('gravity-base').textContent = `${local_g.toFixed(4)} m/s²`;
        $('gps-toggle-btn').innerHTML = '▶️ MARCHE GPS'; 
        
        // Démarrer la boucle lente (Horloge/Astro)
        startSlowLoop();

        // L'état initial est en PAUSE, attendant le clic de l'utilisateur
        stopGPSAcquisition(); 
        stopSensorListeners(); 
        stopIMUSensors();
        
        console.log("Système GNSS Dashboard est prêt. Cliquez sur '▶️ MARCHE GPS' pour démarrer la fusion UKF.");
    });
})(window);
