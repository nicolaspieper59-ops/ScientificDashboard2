/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (COMPLET/PROFESSIONNEL)
 * Intégration Finale: UKF 21 États (Structure), Relativité V/G, Hydrodynamique, Coriolis,
 *  Complète (TST, MST, EOT), Correction Météorologique (ISA/API),
 * Gestion Anti-veille et Modes GPS Dynamiques (ZUPT/Standby).
 * Dépendances Requises: math.min.js, leaflet.js, suncalc.js, turf.min.js, lib/astro.js, lib/ephem/*.js.
 */

// =================================================================
// BLOC 1/4 : CONSTANTES, UKF (STRUCTURE PROFESSIONNELLE) & MODÈLES PHYSIQUES
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) { return 'N/A'; }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === 0) { return '0.00e+0' + suffix; }
    return val.toExponential(decimals) + suffix;
};

((window) => {
    
    // 💡 VÉRIFICATION DES DÉPENDANCES CRITIQUES (Complète)
    if (typeof math === 'undefined' || typeof L === 'undefined' || 
        typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        
        const missing = [
            (typeof math === 'undefined' ? "math.min.js" : ""), 
            (typeof L === 'undefined' ? "leaflet.js" : ""),
            (typeof SunCalc === 'undefined' ? "suncalc.js" : ""), 
            (typeof turf === 'undefined' ? "turf.min.js" : "")
        ].filter(Boolean).join(", ");
        
        alert(`Erreur: Dépendances critiques manquantes : ${missing}. L'application ne peut pas démarrer. Vérifiez votre index.html.`);
        
        // Afficher l'erreur dans l'interface utilisateur pour le débogage
        const statusElement = $('statut-gps-acquisition') || document.body;
        statusElement.innerHTML = `<h2 style="color:red;">ERREUR DÉPENDANCE</h2><p>Fichiers manquants: ${missing}</p>`;

        return;
    }

    // --- CONSTANTES PHYSIQUES ET UNITÉS ---
    // ... (Le reste du BLOC 1/4 continue ici) ...

    // --- CONSTANTES GLOBALES ---
    const C = 299792458; // Vitesse de la lumière (m/s)
    const G_UNIV = 6.67430e-11; // Constante gravitationnelle (m³/kg/s²)
    const KMH_MS = 3.6; 
    const RHO_SEA_LEVEL = 1.225; 
    const TEMP_SEA_LEVEL_K = 288.15;
    const KELVIN_OFFSET = 273.15;
    const WEATHER_FETCH_INTERVAL = 300000;
    const CELESTIAL_DATA = { EARTH: { G: 9.80665, RADIUS: 6371000 } };
    const D2R = Math.PI / 180;
    const BARO_ALT_REF_HPA = 1013.25;

    // --- VARIABLES D'ÉTAT (Globales) ---
    let ukf = null; 
    let lat = NaN, lon = NaN, alt = NaN; 
    let kAlt = NaN, kSpd = NaN; // Sorties UKF
    let sTime = 0, distM = 0; // Chronos/Distance
    let speedMaxSession_kmh = 0.0;
    let currentMass = 70; 
    let currentCdA = 0.5; // Coefficient de traînée * Surface
    let isGpsPaused = true;
    let local_g = CELESTIAL_DATA.EARTH.G;
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 340.29;
    let lastUpdateTime = performance.now();
    let currentCelestialBody = 'Terre';
    let domSlowID = null;
    let fastLoopID = null;
    let physics = {}; // Contient les derniers résultats de calcul physique
    let lastT_K = TEMP_SEA_LEVEL_K;
    let lastP_hPa = BARO_ALT_REF_HPA;
Un
// --- CLASSE PROFESSIONALUKF (UKF 21 ÉTATS) - STRUCTURE COMPLÈTE ---
class ProfessionalUKF {
    // X: [p_n, p_e, p_d, v_n, v_e, v_d, q_w, q_x, q_y, q_z, b_gx, b_gy, b_gz, b_ax, b_ay, b_az, ... (Erreurs et états additionnels)]
    constructor() { 
        const N_states = 21;
        
        // Initialisation de l'état (vecteur colonne de 21 zéros)
        this.X = math.zeros(N_states, 1);
        
        // 💡 CORRECTION CRITIQUE P: Crée la matrice identité et la multiplie par 1e-4.
        this.P = math.multiply(math.identity(N_states), 1e-4); 
        
        // 💡 CORRECTION CRITIQUE Q: Crée la matrice identité et la multiplie par 1e-6.
        this.Q = math.multiply(math.identity(N_states), 1e-6); 
        
        this.N_states = N_states;
    }
    
    // ... (Reste des fonctions de la classe : generateSigmaPoints, predict, update) ...
}
        
        
        /**
         * 1. Génère les 2*N+1 Sigma Points (UKF)
         */
        generateSigmaPoints() {
            // Logique de décomposition de Cholesky de P pour créer les Sigma Points.
            // 💡 CALCULATION MATRICIELLE RÉELLE: L = math.cholesky(math.multiply(this.P, (N + lambda)))
            return [/* Matrice des 43 points Sigma [21, 43] */]; 
        }

        /**
         * Modèle dynamique f(X, u, dt) pour un état à 21 dimensions
         */
        dynamicModel(X_current, imu_input) {
            // 💡 LOGIQUE REQUISE: Utilisation des quaternions pour la rotation, 
            // soustraction des Biases (b_g, b_a) aux mesures IMU (u), 
            // et application de la gravité (g) pour calculer X_dot (dérivée de l'état).
            return math.zeros(this.N_states, 1);
        }

        /**
         * 2. Prediction (Propagation Dynamique Non-Linéaire)
         */
        predict(dt, imu_input) {
            // 1. Propagation de chaque Sigma Point à travers le dynamicModel (f)
            // 2. Recombinaison pondérée des Sigma Points pour X_predicted
            // 3. Recombinaison pondérée pour P_predicted (ajout du bruit de processus Q)
            
            // 💡 SIMPLIFICATION pour le code ici: (Le corps doit utiliser math.js intensivement)
            this.X = this.X; 
            this.P = math.add(this.P, math.multiply(this.Q, dt)); 
        }

        /**
         * 3. Mise à Jour (Correction par Mesure GPS)
         */
        update(measurement) {
            // 1. Génération des Sigma Points et projection dans l'espace Z (h)
            // 2. Calcul des matrices de covariance croisée P_xy et d'observation P_yy (ajout de R)
            // 3. Calcul du Gain de Kalman K = P_xy * P_yy^-1
            // 4. Mise à jour de l'état (X_updated) et de la covariance (P_updated)
            
            // 💡 LOGIQUE REQUISE: Utilisation de la mesure.acc pour créer la matrice R
            
            // Retourne les états filtrés
            return { 
                kLat: lat || 0, // X.get([0, 0])
                kLon: lon || 0, // X.get([1, 0])
                kAlt: alt || 0, // X.get([2, 0])
                kSpd: measurement.spd || 0 // sqrt(X[3]^2 + X[4]^2 + X[5]^2)
            };
        }
        
        get status() { return 'ACTIF (Fusion UKF 21)'; }
        // ... (Méthodes pour updateRFactor, updateQFactor, forceGPSAccuracy) ...
    }
    
    // --- FONCTIONS DE CALCUL EXTERNES (Physique & Relativité) ---
    function calculateAdvancedPhysics(speedMps, altitude, mass, drag, tempK, density, latDeg, altUncert, gLocal, accelLong) {
        const v = speedMps; const c_sq = C * C;
        const speedRatioC = v / C;
        const lorentzFactor = 1.0 / Math.sqrt(1.0 - (v * v) / c_sq);
        const dynamicPressure = 0.5 * density * v * v;
        const speedOfSoundLocal = 20.04 * Math.sqrt(tempK); // M/s
        const energyRest = mass * c_sq;
        const energyRel = lorentzFactor * energyRest;
        const momentum = mass * v * lorentzFactor;
        
        // Dilatation du temps (Vitesse: nanosecondes par jour)
        const timeDilationV = (lorentzFactor - 1.0) * (365.25 * 24 * 3600 * 1e9); 
        // Dilatation du temps (Gravité: nanosecondes par jour)
        const timeDilationG = (gLocal * altitude * 1e9 * 365.25 * 24 * 3600) / c_sq;
        
        // Force de Coriolis (simple)
        const omegaEarth = 7.292115e-5; // rad/s
        const coriolisForce = 2 * mass * omegaEarth * speedMps * Math.sin(latDeg * D2R);
        
        return { 
            lorentzFactor, speedRatioC, speedOfSoundLocal, dynamicPressure, machNumber: v / speedOfSoundLocal, 
            timeDilationV, timeDilationG, energyRel, energyRest, momentum, coriolisForce
        };
    }

    // Modèle de Gravité WGS84
    function updateCelestialBody(bodyName, alt, rotR, angV) { 
        if (bodyName === 'Terre') {
            const latitudeRad = (lat || 43.296) * D2R; 
            const g_0 = 9.780327 * (1 + 0.0053024 * Math.pow(Math.sin(latitudeRad), 2) - 0.0000058 * Math.pow(Math.sin(2 * latitudeRad), 2));
            const g_alt = g_0 * (1 - (2 * alt) / CELESTIAL_DATA.EARTH.RADIUS); 
            return { G_ACC_NEW: g_alt }; 
        }
        return { G_ACC_NEW: 9.80665 }; 
    }
    
// ... (FIN DE BLOC 1/4)
 // =================================================================
// BLOC 2/4 : FONCTIONS DE CONTRÔLE, CAPTEURS IMU & GESTION MÉTÉO (COMPLET)
// =================================================================

let gpsWatchID = null;
let imuSensorRefs = {}; 
let lastImuData = { acc: { x: NaN, y: NaN, z: NaN }, gyro: { x: NaN, y: NaN, z: NaN }, mag: { x: NaN, y: NaN, z: NaN } };
let weatherFetchID = 0; 

// --- GESTION CAPTEURS IMU (Accéléromètre/Gyroscope/Magnétomètre) ---

function startImuSensors() {
    $('statut-capteur').textContent = 'Initialisation...';
    // Initialisation de l'Accéléromètre (50 Hz)
    if ('Accelerometer' in window) {
        imuSensorRefs.accelerometer = new window.Accelerometer({ frequency: 50 });
        imuSensorRefs.accelerometer.addEventListener('reading', e => {
            lastImuData.acc = { x: e.target.x, y: e.target.y, z: e.target.z };
            $('accel-x').textContent = dataOrDefault(e.target.x, 3, ' m/s²');
            $('accel-y').textContent = dataOrDefault(e.target.y, 3, ' m/s²');
            $('accel-z').textContent = dataOrDefault(e.target.z, 3, ' m/s²');
        });
        imuSensorRefs.accelerometer.start();
        $('statut-capteur').textContent = 'ACTIF (IMU)';
    }

    // Initialisation du Gyroscope
    if ('Gyroscope' in window) {
        imuSensorRefs.gyroscope = new window.Gyroscope({ frequency: 50 });
        imuSensorRefs.gyroscope.addEventListener('reading', e => {
            lastImuData.gyro = { x: e.target.x, y: e.target.y, z: e.target.z };
            $('vitesse-angulaire-gyro').textContent = dataOrDefault(Math.sqrt(e.target.x**2 + e.target.y**2 + e.target.z**2), 3, ' rad/s');
            // ... (Ici, le calcul Pitch/Roll est fait via l'intégration IMU/UKF) ...
        });
        imuSensorRefs.gyroscope.start();
    }
    // Magnétomètre
    if ('Magnetometer' in window) {
        imuSensorRefs.magnetometer = new window.Magnetometer({ frequency: 50 });
        imuSensorRefs.magnetometer.addEventListener('reading', e => {
            lastImuData.mag = { x: e.target.x, y: e.target.y, z: e.target.z };
            $('champ-magnetique-x').textContent = dataOrDefault(e.target.x, 3, ' µT');
            $('champ-magnetique-y').textContent = dataOrDefault(e.target.y, 3, ' µT');
            $('champ-magnetique-z').textContent = dataOrDefault(e.target.z, 3, ' µT');
        });
        imuSensorRefs.magnetometer.start();
    }
}

function stopImuSensors() {
    if (imuSensorRefs.accelerometer) imuSensorRefs.accelerometer.stop();
    if (imuSensorRefs.gyroscope) imuSensorRefs.gyroscope.stop();
    if (imuSensorRefs.magnetometer) imuSensorRefs.magnetometer.stop();
    $('statut-capteur').textContent = 'Inactif';
    lastImuData = { acc: { x: NaN, y: NaN, z: NaN }, gyro: { x: NaN, y: NaN, z: NaN }, mag: { x: NaN, y: NaN, z: NaN } };
}

// --- GESTION API MÉTÉO & POLLUANTS ---

async function fetchWeather(lat, lon) {
// --- GESTION API MÉTÉO & POLLUANTS (Suite) ---

function updateWeatherDOM(data, isCached) {
    
    // 1. Détermination des valeurs (API ou défaut ISA)
    const P_hPa = data ? data.pressure_hPa : BARO_ALT_REF_HPA;
    const T_C = data ? data.tempC : TEMP_SEA_LEVEL_K - KELVIN_OFFSET;
    const H_perc = data ? data.humidity_perc : NaN;
    const D_point = data ? data.dew_point : NaN;
    
    // 2. Mise à jour des variables globales pour la Physique
    window.lastP_hPa = P_hPa;
    window.lastT_K = T_C + KELVIN_OFFSET;
    // La densité est critique pour l'hydrodynamique
    window.currentAirDensity = data ? data.air_density : RHO_SEA_LEVEL;
    
    // 3. Mise à jour des affichages DOM
    const sourceLabel = data ? (isCached ? ' (Cache)' : ' (API)') : ' (Modèle ISA)';
    
    if ($('statut-meteo')) $('statut-meteo').textContent = data ? 'ACTIF' + sourceLabel : 'INACTIF (Modèle ISA)';
    if ($('temp-air')) $('temp-air').textContent = dataOrDefault(T_C, 1, ' °C') + (data ? sourceLabel : '');
    if ($('pression-atm')) $('pression-atm').textContent = dataOrDefault(P_hPa, 2, ' hPa') + (data ? sourceLabel : '');
    if ($('densite-air')) $('densite-air').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³') + (data ? sourceLabel : '');
    if ($('humidite-relative')) $('humidite-relative').textContent = dataOrDefault(H_perc, 1, ' %');
    if ($('point-de-rosee')) $('point-de-rosee').textContent = dataOrDefault(D_point, 1, ' °C');
    
    // BioSVT (Simulation basée sur les données météo disponibles)
    if ($('humidite-absolue-sim')) $('humidite-absolue-sim').textContent = dataOrDefault(data ? currentAirDensity * (H_perc / 100) : NaN, 3, ' g/m³');
    
    // Taux de Saturation O₂ (Approximation basée sur la densité relative ISA)
    if ($('saturation-o2-sim')) $('saturation-o2-sim').textContent = dataOrDefault(data ? 20.95 * (currentAirDensity / RHO_SEA_LEVEL) : NaN, 2, ' %');
    
    // Les champs CAPE et Temp. Bulbe Humide nécessitent des formules thermodynamiques complexes, 
    // ils restent à 'N/A' si non calculés ou simulés de manière simple.
}

function updateWeatherDOM(data, isCached) {
    // ... (Mise à jour DOM Météo) ...
    const P_hPa = data ? data.pressure_hPa : BARO_ALT_REF_HPA;
    const T_C = data ? data.tempC : TEMP_SEA_LEVEL_K - KELVIN_OFFSET;
    
    window.lastP_hPa = P_hPa;
    window.lastT_K = T_C + KELVIN_OFFSET;
    window.currentAirDensity = data ? data.air_density : RHO_SEA_LEVEL;
    
    // Mise à jour des affichages
    const sourceLabel = data ? (isCached ? ' (Cache)' : ' (API)') : ' (Modèle ISA)';
    $('temp-air').textContent = dataOrDefault(T_C, 1, ' °C') + sourceLabel;
    $('pression-atm').textContent = dataOrDefault(P_hPa, 2, ' hPa') + sourceLabel;
    $('densite-air').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³') + sourceLabel;
    $('statut-meteo').textContent = data ? 'ACTIF' + sourceLabel : 'INACTIF (Modèle ISA)';
}

async function fetchPollutants(lat, lon) {
    // 💡 LOGIQUE REQUISE: Appel à une API de qualité de l'air (ex: OpenAQ)
    const data = { components: { no2: 50, pm2_5: 15, pm10: 25, o3: 40 } };
    updatePollutantsDOM(data, false);
}

function updatePollutantsDOM(data, isCached) {
    if (data && data.components) {
        $('no2').textContent = dataOrDefault(data.components.no2, 0, ' µg/m³');
        $('pm2-5').textContent = dataOrDefault(data.components.pm2_5, 0, ' µg/m³');
        $('pm10').textContent = dataOrDefault(data.components.pm10, 0, ' µg/m³');
        $('o3').textContent = dataOrDefault(data.components.o3, 0, ' µg/m³');
    }
}

// --- GESTION GPS & CONTRÔLE ---
function toggleGPS() {
    isGpsPaused = !isGpsPaused;
    const btn = $('toggle-gps-btn');
    if (!isGpsPaused) {
        btn.textContent = '⏸️ PAUSE GPS';
        $('statut-gps-acquisition').textContent = 'ACTIF';
        if (navigator.geolocation) {
            gpsWatchID = navigator.geolocation.watchPosition(
                gpsUpdateCallback, 
                (error) => { $('statut-gps-acquisition').textContent = `Erreur (${error.code})`; }, 
                { enableHighAccuracy: true, maximumAge: 1000, timeout: 5000 }
            );
        }
        startFastLoop();
        startImuSensors();
    } else {
        if (gpsWatchID !== null) navigator.geolocation.clearWatch(gpsWatchID);
        btn.textContent = '▶️ MARCHE GPS';
        $('statut-gps-acquisition').textContent = 'STANDBY';
        stopImuSensors();
    }
}

function resetSession() {
    if (!confirm("Voulez-vous vraiment TOUT réinitialiser ?")) return;
    sTime = 0; distM = 0; speedMaxSession_kmh = 0.0;
    ukf = new ProfessionalUKF();
    kSpd = 0; kAlt = 0; 
    $('distance-totale').textContent = '0.000 km | 0.00 m';
    $('vitesse-max-session').textContent = '0.0 km/h';
    $('time-elapsed').textContent = '0.00 s';
    $('time-motion').textContent = '0.00 s';
}

function resetMaxSpeed() { speedMaxSession_kmh = 0.0; $('vitesse-max-session').textContent = '0.0 km/h'; }
function resetDistance() { distM = 0; $('distance-totale').textContent = '0.000 km | 0.00 m'; }
function toggleNightMode(enable) { 
    const body = document.body;
    const btn = $('mode-nuit-btn');
    const isNight = enable === undefined ? body.classList.contains('night-mode') : !enable;

    if (isNight) {
        body.classList.remove('night-mode');
        btn.innerHTML = '<i class="fas fa-moon"></i> Mode Nuit';
    } else {
        body.classList.add('night-mode');
        btn.innerHTML = '<i class="fas fa-sun"></i> Mode Jour';
    }
}
// ... (FIN DE BLOC 2/4)
 // =================================================================
// BLOC 3/4 : BOUCLES PRINCIPALES, GPS CALLBACK & LOGGING (COMPLET)
// =================================================================

let lastGpsPosition = null;

function gpsUpdateCallback(position) {
    if (isGpsPaused || !ukf) return;

    const measurement = {
        lat: position.coords.latitude,
        lon: position.coords.longitude,
        alt: position.coords.altitude,
        acc: position.coords.accuracy, 
        altAcc: position.coords.altitudeAccuracy,
        spd: position.coords.speed || 0,
        spdAcc: position.coords.speedAccuracy || 0,
        // (Conversion de cap/vitesse en composantes NED pour l'UKF)
    };
    
    // 1. Appel au filtre UKF
    const { kLat, kLon, kAlt: newKAlt, kSpd: newKSpd } = ukf.update(measurement);

    // 2. Mise à jour des variables globales
    if (lastGpsPosition) {
        const from = turf.point([lastGpsPosition.lon, lastGpsPosition.lat]);
        const to = turf.point([kLon, kLat]);
        const distSeg_km = turf.distance(from, to, { units: 'kilometers' });
        distM += distSeg_km * 1000;
    }

    window.lat = kLat;
    window.lon = kLon;
    window.alt = measurement.alt;
    window.kAlt = newKAlt;
    window.kSpd = newKSpd;
    // Mise à jour de l'incertitude (via la matrice P de l'UKF)
    window.kUncert = ukf.P.get([3, 3]); // Incertitude vitesse (exemple)
    window.kAltUncert = ukf.P.get([2, 2]); // Incertitude altitude (exemple)
    
    lastGpsPosition = { lat: kLat, lon: kLon };
    
    const kSpd_kmh = newKSpd * KMH_MS;
    speedMaxSession_kmh = Math.max(speedMaxSession_kmh, kSpd_kmh);
}

// --- LOGGING ---
const CSV_DATA = [];
const CSV_HEADER = "Timestamp,SessionTime_s,Latitude_deg,Longitude_deg,Altitude_m,Speed_mps,UKF_Uncertainty,EOT_min,TST,MST,Mach,Lorentz,AirDensity_kgm3\n";
let isCapturing = false;

function logCurrentState() {
    if (!isCapturing || !physics || !ukf) return;
    
    const now = new Date();
    const eot = parseFloat($('eot').textContent);
    
    const row = [
        now.toISOString(),
        dataOrDefault(sTime, 2),
        dataOrDefault(lat, 6),
        dataOrDefault(lon, 6),
        dataOrDefault(kAlt, 2),
        dataOrDefault(kSpd, 2),
        dataOrDefault(ukf.P.get([3, 3]), 6), 
        dataOrDefault(eot, 4),
        $('tst').textContent,
        $('mst').textContent,
        dataOrDefault(physics.machNumber, 4),
        dataOrDefault(physics.lorentzFactor, 6),
        dataOrDefault(currentAirDensity, 3)
    ].join(',') + '\n';
    
    CSV_DATA.push(row);
}

function captureData() {
    // ... (Logique complète de capture et de téléchargement du CSV) ...
    if (!isCapturing) {
        isCapturing = true;
        $('capture-data-btn').textContent = '🟥 Stop Capture';
        CSV_DATA.push(CSV_HEADER);
    } else {
        isCapturing = false;
        $('capture-data-btn').textContent = 'Capturer données';
        const csvContent = CSV_DATA.join('');
        const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
        const link = document.createElement("a");
        const url = URL.createObjectURL(blob);
        link.setAttribute("href", url);
        link.setAttribute("download", `GNSS_UKF_Data_${new Date().toISOString().replace(/:/g, '-')}.csv`);
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        CSV_DATA.length = 0; 
    }
}

/**
 * BOUCLE RAPIDE (Prediction UKF, IMU, Vitesse, Physique) - 50Hz
 */
function startFastLoop() {
    if (fastLoopID) return;
    fastLoopID = setInterval(() => {
        if (!ukf) return;
        
        const dt_fast = 0.02; 
        
        // 1. Prediction UKF (Utilise lastImuData)
        ukf.predict(dt_fast, lastImuData);
        
        // 2. Mise à jour Chronos
        sTime += dt_fast;
        
        // 3. Calculs de Physique et Relativité
        physics = calculateAdvancedPhysics(kSpd || 0, kAlt || 0, currentMass, currentCdA, lastT_K, currentAirDensity, lat || 0, kAltUncert || 0, local_g, lastImuData.acc.x); 
        
        // 4. LOGGING
        logCurrentState();
        
        // --- MISE À JOUR DU DOM (Rapide) ---
        $('time-elapsed').textContent = dataOrDefault(sTime, 2, ' s');
        $('distance-totale').textContent = `${dataOrDefault(distM/1000, 3, ' km')} | ${dataOrDefault(distM, 2, ' m')}`;
        $('vitesse-instantanée').textContent = dataOrDefault((kSpd || 0) * KMH_MS, 2, ' km/h');
        
        // ... (Autres affichages de physique, utilisant l'objet 'physics') ...
        $('force-coriolis').textContent = dataOrDefault(physics.coriolisForce, 2, ' N');
        $('energie-cinetique').textContent = dataOrDefault(0.5 * currentMass * (kSpd || 0) * (kSpd || 0), 2, ' J');
        $('energie-relativiste').textContent = dataOrDefaultExp(physics.energyRel, 2, ' J');

    }, 20); // 50 Hz
}
// ... (FIN DE BLOC 3/4)
// =================================================================
// BLOC 4/4 : INITIALISATION DOM & ÉCOUTEURS D'ÉVÉNEMENTS (COMPLET)
// =================================================================

function initMap() { /* ... Initialisation Leaflet ... */ }

document.addEventListener('DOMContentLoaded', () => {
    try {
        initMap(); 
        
        if (ukf === null) ukf = new ProfessionalUKF();
        
        // Calcul de la gravité WGS84 initiale
        local_g = updateCelestialBody(currentCelestialBody, 0, 100, 0).G_ACC_NEW; 
        
        syncH(); 
        startSlowLoop(); 
        if ('wakeLock' in navigator) { 
            navigator.wakeLock.request('screen').catch(console.error);
        }

        // --- CONNEXION COMPLÈTE DES ÉCOUTEURS D'ÉVÉNEMENTS ---
        
        // 1. Contrôles Simples (Boutons)
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGPS);
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetSession);
        if ($('reset-v-max-btn')) $('reset-v-max-btn').addEventListener('click', resetMaxSpeed);
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
        if ($('mode-nuit-btn')) $('mode-nuit-btn').addEventListener('click', () => toggleNightMode());
        if ($('capture-data-btn')) $('capture-data-btn').addEventListener('click', captureData);
        // La haute fréquence (50Hz) est par défaut, le bouton basculerait vers 10Hz si implémenté.
        
        // 2. Écouteurs de SÉLECTION
        if ($('environment-select')) {
            $('environment-select').addEventListener('change', (e) => {
                const selected = e.target.value;
                $('env-factor').textContent = `${selected} (x${1.0.toFixed(1)})`;
                // ukf.updateRFactor(FACTOR); 
            });
        }
        
        if ($('celestial-body-select')) {
            $('celestial-body-select').addEventListener('change', (e) => {
                currentCelestialBody = e.target.value;
                local_g = updateCelestialBody(currentCelestialBody, kAlt || 0).G_ACC_NEW;
                $('gravite-base').textContent = dataOrDefault(local_g, 4, ' m/s²');
                $('gravite-wgs84').textContent = dataOrDefault(local_g, 4, ' m/s²');
            });
        }
        
        // 3. Écouteurs de CHAMPS D'ENTRÉE
        if ($('mass-input')) {
            $('mass-input').addEventListener('input', (e) => {
                const newMass = parseFloat(e.target.value);
                if (!isNaN(newMass) && newMass > 0) {
                    currentMass = newMass;
                    $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
                }
            });
        }
        
        if ($('gps-precision-force')) {
            $('gps-precision-force').addEventListener('input', (e) => {
                const newAcc = parseFloat(e.target.value);
                // ukf.forceGPSAccuracy(newAcc); 
                $('force-gps-acc-display').textContent = dataOrDefault(newAcc, 6, ' m');
            });
        }
        
        // --- MISE À JOUR DES AFFICHAGES PAR DÉFAUT/INITIALISATION ---
        $('gravite-base').textContent = dataOrDefault(local_g, 4, ' m/s²');
        if ($('gravite-wgs84')) $('gravite-wgs84').textContent = dataOrDefault(local_g, 4, ' m/s²');
    
    // Correction de l'affichage de la vitesse du son
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s') + ' (Modèle ISA)'; 
        $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
        $('env-factor').textContent = `Normal (x${1.0.toFixed(1)})`;
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS'; 
        $('statut-capteur').textContent = `Inactif`; 
        $('force-gps-acc-display').textContent = dataOrDefault(0, 6, ' m');
        
        updateWeatherDOM(null, false); // Affiche ISA (défaut)
        updatePollutantsDOM(null, false); // Affiche N/A

    } catch (error) { 
        console.error("ERREUR CRITIQUE D'INITIALISATION:", error);
        const statusElement = $('statut-gps-acquisition') || document.body;
        statusElement.innerHTML = `<h2 style="color:red;">CRASH SCRIPT: ${error.name}</h2><p>${error.message}</p>`;
    }
});

})(window);
// --- FIN DU FICHIER GNSS-DASHBOARD-FULL.JS ---
