// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Partie 1/4 : Constantes, État Global, Utilitaires & UKF Core
// =================================================================

// --- CLÉS D'API & ENDPOINTS ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_CONST = 6.67430e-11; // Constante gravitationnelle (m³/kg/s²)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Facteur de conversion
const AU_TO_M = 149597870700; 
const LIGHT_YEAR_TO_M = 9.461e15; 
const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (hPa)
const TEMP_SEA_LEVEL_K = 288.15; // Température de référence (15°C)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air de référence (kg/m³)

// --- PARAMÈTRES DU FILTRE DE KALMAN (UKF 21 États: Pos/Vel/Ori/Bias IMU/G-Vector) ---
const UKF_STATE_DIM = 21;   
let R_NOISE_OVERRIDE = 0.0; // Précision GPS forcée (m)
let UKF_REACTIVITY = 0.5;   // Facteur de fusion (gain Kalman)

// --- CONFIGURATIONS ENVIRONNEMENTALES (Facteur de Bruit R) ---
const ENVIRONMENT_FACTORS = {
    NORMAL: { MULT: 1.0, DISPLAY: "Normal (x1.0)", R_MULT: 1.0 },
    FOREST: { MULT: 1.0, DISPLAY: "Forêt (x2.5)", R_MULT: 2.5 }, 
    CONCRETE: { MULT: 1.0, DISPLAY: "Grotte/Tunnel (x7.0)", R_MULT: 7.0 },
    METAL: { MULT: 1.0, DISPLAY: "Métal/Bâtiment (x5.0)", R_MULT: 5.0 }
};
const CELESTIAL_BODIES = {
    EARTH: { G_REF: 9.8067, R: 6371000, M: 5.972e24 },
    MOON: { G_REF: 1.62, R: 1737400, M: 7.34767e22 },
    MARS: { G_REF: 3.71, R: 3389500, M: 6.4171e23 },
    ROTATING: { G_REF: 0.0, R: 0, M: 0 } 
};

// --- CONFIGURATIONS DES DELAIS D'UPDATE ---
const GPS_UPDATE_INTERVAL = 100; // ms (Fréquence du capteur)
const DOM_SLOW_UPDATE = 333; // ms (Mise à jour de l'interface)
const WEATHER_UPDATE_INTERVAL = 1800000; // 30 minutes
const NTP_SYNC_INTERVAL = 3600000; // 1 heure

// =================================================================
// ÉTAT GLOBAL DU SYSTÈME (Variables de Fusion et Brutes)
// =================================================================

let isGpsRunning = false;
let netherMode = false; 
let distanceRatioMode = false; 
let currentEnvironment = 'NORMAL';
let currentCelestialBody = 'EARTH';
let watchId = null;
let lastKnownPos = null;

// Position de travail (GPS/IMU Brutes)
let currentPosition = { lat: 43.2964, lon: 5.3697, alt: 0.0, acc: 10.0, speed: 0.0, heading: 0.0 };

// Variables UKF/EKF (Estimées/Filtrées) - Représentant X_k
let kLat = currentPosition.lat, kLon = currentPosition.lon, kAlt = currentPosition.alt;
let kSpd = 0.0; // Vitesse stable (m/s)
let kAltDt = 0.0; // Vitesse verticale (m/s)
let kHeading = 0.0; // Cap stable (rad)
let lastTimestamp = 0; 
let ukfRNoise = 100.0; // Bruit de mesure R réel (m)

// Variables IMU (Synthétiques/Non-filtrées)
let imu = { ax: 0.0, ay: 0.0, az: 9.81, mx: 0, my: 0, mz: 0, pitch: 0, roll: 0, yaw: 0, available: true };

// Variables de Trajectoire
let maxSpdMs = 0; 
let distM = 0; 
let startTimeMs = new Date().getTime(); 
let motionTimeSec = 0; 
let totalTimeSec = 0; 

// Correction Temporelle (NTP)
let lServH = 0; 
let lLocH = 0;
let ntpDelayMs = 0;

// Variables Météorologiques et Environnementales
let lastWeather = { P_hPa: BARO_ALT_REF_HPA, T_K: TEMP_SEA_LEVEL_K, H_perc: 0.50, airDensity: RHO_SEA_LEVEL, speedOfSound: 340.29 };
let lastPollutants = { no2: 0, pm25: 0, pm10: 0, o3: 0 };
let lastWeatherFetch = 0;
let G_ACC_CURRENT = CELESTIAL_BODIES.EARTH.G_REF;
let currentMass = 70.0;
let rotationRadius = 100;
let angularVelocity = 0.0;
let DRAG_COEFF = 0.8; 
let SURFACE_AREA = 0.5; 

// État UKF (Simulation matricielle)
// Bien que le vrai UKF ne soit pas implémentable ici, on simule sa sortie P (covariance)
let UKF_P_k = math.matrix(math.diag(math.ones(UKF_STATE_DIM).map(() => 1e-4))); 

// =================================================================
// FONCTIONS UTILITAIRES GLOBALES
// =================================================================

const $ = id => document.getElementById(id);

const dataOrDefault = (val, decimals, suffix = '', multiplier = 1) => {
    if (val === undefined || val === null || isNaN(val)) return 'N/A' + suffix;
    return (val * multiplier).toFixed(decimals) + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+00' + suffix;
    }
    // S'assurer qu'un petit nombre affiche l'exponentiel correctement si nécessaire
    return val.toExponential(decimals) + suffix;
};

const getCDate = (serverTimeMs, localTimeMsAtSync) => {
    if (serverTimeMs === 0 || localTimeMsAtSync === 0) return null;
    const nowLocal = new Date().getTime();
    return new Date(serverTimeMs + (nowLocal - localTimeMsAtSync));
};

const formatTime = (seconds) => {
    const s = Math.floor(seconds % 60);
    const m = Math.floor((seconds / 60) % 60);
    const h = Math.floor((seconds / 3600));
    return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
};
// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Partie 2/4 : Modèles Physiques et Astro
// =================================================================

// --- Modèles Météorologiques / Densité / Son ---

const R_AIR = 287.058; 
const R_WATER_VAPOR = 461.5; 

const getSpeedOfSound = (tempK) => Math.sqrt(1.4 * R_AIR * tempK); 

const getAirDensity = (pressure_hPa, tempK, humidity_perc) => {
    const P_Pa = pressure_hPa * 100;
    const T_C = tempK - 273.15;
    
    // Pression de vapeur d'eau saturante (Psat_hPa)
    const Psat_hPa = 6.1078 * Math.pow(10, (7.5 * T_C) / (T_C + 237.3));
    const P_v = (Psat_hPa * (humidity_perc / 100.0)) * 100; 
    const P_d = P_Pa - P_v; 
    
    return (P_d / (R_AIR * tempK)) + (P_v / (R_WATER_VAPOR * tempK));
};

// --- Modèles Relativistes ---
const calculateRelativity = (speedMps, massKg) => {
    const v_sq_c_sq = (speedMps * speedMps) / (C_L * C_L);
    const gamma = (v_sq_c_sq >= 1) ? Infinity : 1 / Math.sqrt(1 - v_sq_c_sq);
    
    const E0 = massKg * C_L * C_L;
    const E = gamma * E0;
    const momentum = gamma * massKg * speedMps;
    const timeDilationV_ns_day = (gamma - 1) * 86400 * 1e9;
    
    // Rayon de Schwarzschild (Rs = 2GM/c²)
    const Rs = (2 * G_CONST * massKg) / (C_L * C_L);

    return { 
        gamma, 
        E0, 
        E, 
        momentum, 
        v_c_perc: v_sq_c_sq * 100, // % c²
        timeDilationV_ns_day,
        schwarzschildRadius: Rs
    };
};

const calculateGravity = (bodyKey, altM, rotationR, angularV, latRad) => {
    const body = CELESTIAL_BODIES[bodyKey];
    let G_ACC_NEW = body.G_REF;

    if (bodyKey === 'ROTATING') {
        G_ACC_NEW = rotationR * angularV * angularV; 
    } else if (body.R > 0 && body.M > 0) {
        // Gravité en fonction de l'altitude
        const r_alt = body.R + altM;
        G_ACC_NEW = G_CONST * body.M / (r_alt * r_alt);
        
        // CORRECTION WGS84 (Terre uniquement)
        if (bodyKey === 'EARTH' && latRad !== undefined) {
             const G_EQUATOR = 9.7803253359;
             const G_POLAR = 9.8321849378;
             const K = 0.00193185265241;
             
             // Correction de la force centrifuge
             const G_WGS84_LAT = (G_EQUATOR * (1 + K * Math.sin(latRad) * Math.sin(latRad))) / Math.sqrt(1 - 0.00669437999013 * Math.sin(latRad) * Math.sin(latRad));
             
             // Atténuation due à l'altitude (Approximation)
             G_ACC_NEW = G_ACC_NEW * (G_WGS84_LAT / G_ACC_NEW);
        }
    }
    
    return G_ACC_NEW;
};

// --- Modèles Dynamiques & Forces ---

const calculateDrag = (speedMps, airDensity, dragCoeff, surfaceArea) => {
    const dynamicPressure = 0.5 * airDensity * speedMps * speedMps;
    const dragForce = dynamicPressure * dragCoeff * surfaceArea;
    const dragPower = dragForce * speedMps; 
    
    // Nombre de Reynolds (Re) (Approximation L=1m pour une personne/voiture)
    const REYNOLDS_LENGTH = 1.0; 
    const VISCOSITY = 1.81e-5; // Viscosité dynamique de l'air à 15°C (Pa.s)
    const reynoldsNumber = (airDensity * speedMps * REYNOLDS_LENGTH) / VISCOSITY;

    return {
        dynamicPressure,
        dragForce,
        dragPowerKw: dragPower / 1000, 
        reynoldsNumber
    };
};

const OMEGA_EARTH = 7.2921159e-5; // rad/s
const calculateCoriolis = (speedMps, latRad, massKg, kHeading) => {
    // F_c = -2 * m * (omega x v)
    // Ici, on calcule la composante horizontale (perpendiculaire à la vitesse)
    return 2 * massKg * speedMps * OMEGA_EARTH * Math.sin(latRad);
};

// --- Modèles Astronomiques ---

const updateAstroTime = (lat, lon, now) => {
    const pos = SunCalc.getPosition(now, lat, lon);
    const times = SunCalc.getTimes(now, lat, lon);
    const moon = SunCalc.getMoonPosition(now, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    
    // Angle horaire (H)
    const H_rad = pos.hourAngle; 
    
    // Temps Sidéral Local Vrai (TSLV) (simplification via SunCalc pour l'angle horaire)
    // TSLV = H + Ascension Droite
    
    // TST (Heure Solaire Vraie): Angle Horaire du Soleil converti en Heure
    const TST_Hours = ((H_rad * R2D) + 180) / 360 * 24; // 0h à 24h
    const TST_Sec = TST_Hours * 3600;
    
    const TST_Date = new Date(0, 0, 0, 0, 0, TST_Sec);
    const TST_Time = TST_Date.toUTCString().split(' ')[4];

    // MST (Heure Solaire Moyenne)
    const longitudeHours = lon / 15;
    const MST_Sec = (now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds()) + (longitudeHours * 3600);
    const MST_Date = new Date(0, 0, 0, 0, 0, MST_Sec % 86400);
    const MST_Time = MST_Date.toUTCString().split(' ')[4];
    
    // EOT (Équation du Temps) : TST - MST (en minutes)
    const EOT_Min = (TST_Sec - (MST_Sec % 86400)) / 60;
    
    // Durée du Jour
    const dayDurationSec = (times.sunset.getTime() - times.sunrise.getTime()) / 1000;

    return {
        TST: TST_Time,
        MST: MST_Time,
        EOT: EOT_Min,
        noonSolar: times.solarNoon ? times.solarNoon.toLocaleTimeString('fr-FR', { timeStyle: 'medium' }) : 'N/A',
        sunAlt: pos.altitude * R2D,
        sunAzimuth: pos.azimuth * R2D,
        dayDuration: dayDurationSec,
        moon: moon,
        moonIllumination: moonIllumination
    };
};
// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Partie 3/4 : Fusion Capteurs, UKF & IMU Synthétique
// =================================================================

let map, marker, watchId = null;

// --- Démarrage/Gestion de l'UKF (Simulateur de Filtre) ---
const runUKFStep = (deltaT, measurement) => {
    // --- 1. Matrice de Bruit de Mesure (R) ---
    const envFactorR = ENVIRONMENT_FACTORS[currentEnvironment].R_MULT;
    const R_GPS_m = R_NOISE_OVERRIDE > 0 ? R_NOISE_OVERRIDE : (measurement.acc * envFactorR || 1.0 * envFactorR);
    
    // Facteur de fusion basé sur la précision GPS (R) vs. l'incertitude du modèle (P)
    // UKF_REACTIVITY = P / (P + R) (Simplification)
    const R_INV_COV_POS = 1.0 / (R_GPS_m * R_GPS_m);
    // On simule une amélioration de la confiance si la précision GPS est bonne
    const UKF_GAIN = Math.max(0.01, Math.min(0.9, 0.5 + (10.0 - R_GPS_m) / 10.0 * 0.4));
    UKF_REACTIVITY = UKF_GAIN;

    // --- 2. Lissage Alpha-Beta-Gamma (Simulateur UKF/EKF) ---
    // Représentation de l'étape de mise à jour (K_k * y_tilde)
    
    // Lissage position
    kLat = kLat * (1 - UKF_GAIN) + measurement.lat * UKF_GAIN;
    kLon = kLon * (1 - UKF_GAIN) + measurement.lon * UKF_GAIN;
    
    // Lissage altitude et vitesse verticale
    const altMeasRate = (measurement.alt - kAlt) / deltaT;
    kAltDt = kAltDt * (1 - UKF_GAIN) + altMeasRate * UKF_GAIN;
    kAlt = kAlt * (1 - UKF_GAIN) + measurement.alt * UKF_GAIN;
    
    // Lissage vitesse (horizontal) et cap
    kSpd = kSpd * (1 - UKF_GAIN) + measurement.speed * UKF_GAIN;
    kHeading = kHeading * (1 - UKF_GAIN) + measurement.heading * UKF_GAIN;

    // Mise à jour de la matrice P (Simulation de la réduction de l'incertitude)
    UKF_P_k.set([2, 2], UKF_P_k.get([2, 2]) * (1 - UKF_GAIN * 0.1));
    ukfRNoise = R_GPS_m;
    
    return { kLat, kLon, kAlt, kSpd, kAltDt, kHeading, ukfRNoise };
};

// --- Capteurs IMU Synthétiques (Remplacer la simulation inutile par un modèle physique) ---

const updateSyntheticIMU = (speedMps, massKg, altM, deltaT) => {
    const G = calculateGravity(currentCelestialBody, altM, rotationRadius, angularVelocity, kLat * D2R);

    // 1. Accélération Longitudinale (ax)
    // Basé sur la traînée et une accélération arbitraire d'un moteur/frein
    const { dragForce } = calculateDrag(speedMps, lastWeather.airDensity, DRAG_COEFF, SURFACE_AREA);
    
    // Simulation réaliste : ax = (ForcePropulsion - DragForce) / Mass
    // On simule une petite force de propulsion/freinage si l'utilisateur accélère/décélère fortement (variation de kSpd)
    const syntheticThrustN = (speedMps - kSpd) * massKg * (1 / deltaT) * 0.1; // Force basée sur le différentiel de vitesse filtrée
    imu.ax = (syntheticThrustN - dragForce) / massKg; 

    // 2. Accélération Latérale (ay)
    // Centrifuge due à un virage (simplification)
    const turnRate = (Math.random() * 0.05) * (speedMps > 1 ? 1 : 0); // 0.05 rad/s max turn
    imu.ay = speedMps * turnRate; // Accélération centripète (a = v*w)

    // 3. Accélération Verticale (az)
    // 1g (Gravité) + Accélération verticale réelle (kAltDt_dot)
    const kAltDt_dot = (kAltDt * 0.99 - kAltDt) / deltaT; // Accélération verticale filtrée (très petite)
    imu.az = G / G_ACC_CURRENT + kAltDt_dot; // G/9.81 = 1g environ + Accel verticale

    // 4. Pitch/Roll
    imu.pitch = imu.ax * R2D * 0.5; // Pitch lié à l'accélération longitudinale
    imu.roll = imu.ay * R2D * 1.5; // Roll lié à l'accélération latérale (virage)
    
    // 5. Magnétisme (Brute, non-filtrée)
    // Simuler des fluctuations faibles autour d'une valeur nominale
    const magBase = 45; 
    imu.mx = magBase + Math.random() * 0.5;
    imu.my = magBase + Math.random() * 0.5;
    imu.mz = magBase + Math.random() * 0.5;
};

// --- Gestion des Capteurs GPS (Réel) ---

const startGpsWatch = (options) => {
    if (watchId !== null) navigator.geolocation.clearWatch(watchId);
    
    const handleSuccess = (pos) => {
        if (!isGpsRunning) return;

        const now = pos.timestamp;
        const deltaT = (now - lastTimestamp) / 1000 || (GPS_UPDATE_INTERVAL / 1000); 
        lastTimestamp = now;

        const raw = {
            lat: pos.coords.latitude,
            lon: pos.coords.longitude,
            alt: pos.coords.altitude || kAlt, 
            acc: pos.coords.accuracy || 5.0, // GPS accuracy (m)
            speed: pos.coords.speed || 0.0, // Vitesse (m/s)
            heading: pos.coords.heading || kHeading * R2D 
        };
        raw.heading = raw.heading * D2R; // Convertir en radians pour le filtre

        currentPosition = raw;
        
        // Calcul de la distance totale (turf.js)
        if (lastKnownPos) {
            const from = turf.point([lastKnownPos.lon, lastKnownPos.lat]);
            const to = turf.point([raw.lon, raw.lat]);
            distM += turf.distance(from, to, { units: 'meters' });
        }
        lastKnownPos = { lat: raw.lat, lon: raw.lon };

        maxSpdMs = Math.max(maxSpdMs, raw.speed);

        // --- ÉTAPE UKF (Fusion) ---
        const { kLat, kLon, kAlt, kSpd, ukfRNoise } = runUKFStep(deltaT, raw);
        updateSyntheticIMU(kSpd, currentMass, kAlt, deltaT);

        // Mise à jour de la carte
        if (map && marker) {
            marker.setLatLng([kLat, kLon]);
            // Mise à jour de la rotation du marqueur
            if (kHeading * R2D) {
                marker.options.rotationAngle = kHeading * R2D;
                marker.options.icon.options.html = `<i class="fas fa-location-arrow" style="font-size: 24px; color: #007bff; transform: rotate(${kHeading * R2D}deg);"></i>`;
                marker.setIcon(L.divIcon(marker.options.icon.options));
            }
            map.panTo([kLat, kLon]);
        }
    };

    const handleError = (error) => {
        console.error('Erreur GPS:', error);
        $('statut-gps-acquisition').textContent = `ERREUR: ${error.code}`;
    };

    watchId = navigator.geolocation.watchPosition(handleSuccess, handleError, options);
    $('statut-gps-acquisition').textContent = `ACQUISITION...`;
};

// --- Initialisation de la Carte (Leaflet) ---
const initMap = (lat, lon) => {
    if (typeof L === 'undefined') return;
    map = L.map('map', { zoomControl: false }).setView([lat, lon], 14);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    marker = L.marker([lat, lon], {
        icon: L.divIcon({
            className: 'custom-marker',
            html: '<i class="fas fa-location-arrow" style="font-size: 24px; color: #007bff; transform: rotate(0deg);"></i>',
            iconSize: [24, 24],
            iconAnchor: [12, 12]
        }),
        rotationAngle: 0,
        rotationOrigin: 'center center'
    }).addTo(map);
};
// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Partie 4/4 : Boucle DOM, Météo & Initialisation
// =================================================================

// --- Synchronisation NTP ---
const syncNTP = async () => {
    try {
        const start = new new Date().getTime();
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error("Échec de la synchro NTP");
        const data = await response.json();
        const end = new new Date().getTime();
        
        lServH = data.unixtime * 1000;
        lLocH = end;
        ntpDelayMs = (end - start) / 2;
        $('heure-locale').textContent = getCDate(lServH, lLocH).toLocaleTimeString('fr-FR');
        $('heure-locale').classList.remove('theoretical-crit');
    } catch (e) {
        lServH = 0; lLocH = 0; ntpDelayMs = -1;
        $('heure-locale').textContent = 'SYNCHRO ÉCHOUÉE (LOCALE)';
        $('heure-locale').classList.add('theoretical-crit');
    }
};

// --- Gestion Météo/Polluants (Fetch) ---
const fetchWeather = async (lat, lon) => {
    const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
    try {
        const response = await fetch(url);
        if (!response.ok) throw new Error('API Météo/Air non disponible');
        const data = await response.json();
        
        lastWeather = {
            P_hPa: data.weather.pressure,
            T_K: data.weather.temp + 273.15,
            H_perc: data.weather.humidity / 100.0,
            airDensity: getAirDensity(data.weather.pressure, data.weather.temp + 273.15, data.weather.humidity),
            speedOfSound: getSpeedOfSound(data.weather.temp + 273.15)
        };
        lastPollutants = {
            no2: data.air.no2 || 0, pm25: data.air.pm25 || 0, pm10: data.air.pm10 || 0, o3: data.air.o3 || 0
        };

        $('statut-meteo').textContent = `ACTIF (Dernière MAJ: ${new Date().toLocaleTimeString('fr-FR')})`;
    } catch (e) {
        lastWeather.airDensity = RHO_SEA_LEVEL; // Conserver les valeurs par défaut ISA en cas d'échec
        lastWeather.speedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);
        $('statut-meteo').textContent = 'ÉCHEC (API PROXY)';
    }
};

// --- Boucle d'Update Lente du DOM ---
const startDOMUpdateLoop = () => {
    setInterval(() => {
        const now = getCDate(lServH, lLocH) || new Date();
        const deltaT = (new Date().getTime() - lastTimestamp) / 1000 || (DOM_SLOW_UPDATE / 1000); // Pour les calculs de puissance/énergie
        totalTimeSec = (now.getTime() - startTimeMs) / 1000;
        currentMass = parseFloat($('mass-input').value) || 70;
        
        // Mise à jour de la Gravité Locale
        G_ACC_CURRENT = calculateGravity(currentCelestialBody, kAlt, rotationRadius, angularVelocity, kLat * D2R);

        // --- 1. Calculs & Modèles ---
        const spdKmh = kSpd * KMH_MS;
        const relativity = calculateRelativity(kSpd, currentMass);
        const { dynamicPressure, dragForce, dragPowerKw, reynoldsNumber } = calculateDrag(kSpd, lastWeather.airDensity, DRAG_COEFF, SURFACE_AREA);
        const coriolisForce = calculateCoriolis(kSpd, kLat * D2R, currentMass, kHeading);
        const astro = updateAstroTime(kLat, kLon, now);
        
        // Puissance Mécanique = Force * Vitesse (ici on simule une force nette positive ou négative)
        const mechanicalPower = imu.ax * currentMass * kSpd; 
        
        // --- 2. Mise à jour de l'affichage du temps et des commandes ---
        $('heure-locale').textContent = getCDate(lServH, lLocH).toLocaleTimeString('fr-FR');
        $('date-heure-utc').textContent = now.toISOString().replace('T', ' ').substring(0, 19) + ' UTC';
        $('elapsed-session-time').textContent = formatTime(totalTimeSec);
        $('elapsed-motion-time').textContent = formatTime(motionTimeSec);
        $('time-minecraft').textContent = astro.mcTime;
        $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');

        // Mise à jour de la vitesse et de la distance
        $('speed-stable').textContent = dataOrDefault(spdKmh, 1, ' km/h');
        $('speed-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
        $('speed-stable-kms').textContent = dataOrDefault(kSpd, 6, ' km/s', 1/1000);
        $('vitesse-max-session').textContent = dataOrDefault(maxSpdMs * KMH_MS, 1, ' km/h');
        
        const distanceRatio = netherMode ? 1/8 : (distanceRatioMode ? ((CELESTIAL_BODIES[currentCelestialBody].R + kAlt) / CELESTIAL_BODIES[currentCelestialBody].R) : 1.0);
        const scaledDist = distM * distanceRatio;
        $('distance-totale').textContent = `${dataOrDefault(scaledDist, 3, ' km', 1/1000)} | ${dataOrDefault(distM, 2, ' m')}`;
        $('distance-ratio').textContent = dataOrDefault(distanceRatio, 3);
        $('distance-light-s').textContent = dataOrDefaultExp(scaledDist / C_L, 2, ' s');
        $('distance-light-min').textContent = dataOrDefaultExp(scaledDist / (C_L * 60), 2, ' min');
        $('distance-cosmic').textContent = `${dataOrDefaultExp(scaledDist / AU_TO_M, 2, ' UA')} | ${dataOrDefaultExp(scaledDist / LIGHT_YEAR_TO_M, 2, ' al')}`;

        // Mise à jour Astro
        $('tst').textContent = astro.TST;
        $('mst').textContent = astro.MST;
        $('noon-solar').textContent = astro.noonSolar;
        $('eot').textContent = dataOrDefault(astro.EOT, 2, ' min');
        $('sun-alt').textContent = dataOrDefault(astro.sunAlt, 2, ' °');
        $('sun-azimuth').textContent = dataOrDefault(astro.sunAzimuth, 2, ' °');
        $('day-duration').textContent = formatTime(astro.dayDuration);
        
        // Mise à jour Lune (SunCalc)
        const moonPhaseName = ['Nouvelle Lune', 'Croissant de Lune', 'Premier Quartier', 'Lune Gibbée', 'Pleine Lune', 'Lune Disséminée', 'Dernier Quartier', 'Vieux Croissant'][Math.floor(astro.moonIllumination.phase * 8)];
        $('moon-phase-name').textContent = moonPhaseName;
        $('moon-illuminated').textContent = dataOrDefault(astro.moonIllumination.fraction, 1, ' %', 100);
        $('moon-alt').textContent = dataOrDefault(astro.moon.altitude * R2D, 2, ' °');
        $('moon-azimuth').textContent = dataOrDefault(astro.moon.azimuth * R2D, 2, ' °');
        
        // --- 3. Dynamique, Forces & Relativité ---
        $('gravite-wgs84').textContent = dataOrDefault(G_ACC_CURRENT, 4, ' m/s²');
        $('force-g-long').textContent = dataOrDefault(imu.ax / G_ACC_CURRENT, 2, ' g');
        $('vertical-speed').textContent = dataOrDefault(kAltDt, 2, ' m/s');
        $('force-g-vert').textContent = dataOrDefault(imu.az / G_ACC_CURRENT, 2, ' g');
        
        $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
        $('drag-power-kw').textContent = dataOrDefault(dragPowerKw, 2, ' kW');
        $('reynolds-number').textContent = dataOrDefaultExp(reynoldsNumber, 2);
        $('kinetic-energy').textContent = dataOrDefault(0.5 * currentMass * kSpd * kSpd, 2, ' J');
        $('mechanical-power').textContent = dataOrDefault(mechanicalPower, 2, ' W');
        $('coriolis-force').textContent = dataOrDefault(coriolisForce, 4, ' N');
        
        // Relativité
        $('percent-speed-light').textContent = dataOrDefaultExp(relativity.v_c_perc, 2, ' %');
        $('lorentz-factor').textContent = dataOrDefault(relativity.gamma, 4);
        $('time-dilation-vitesse').textContent = dataOrDefault(relativity.timeDilationV_ns_day, 2, ' ns/j');
        $('relativistic-energy').textContent = dataOrDefaultExp(relativity.E, 2, ' J');
        $('rest-mass-energy').textContent = dataOrDefaultExp(relativity.E0, 2, ' J');
        $('momentum').textContent = dataOrDefaultExp(relativity.momentum, 4, ' kg⋅m/s');
        $('schwarzschild-radius').textContent = dataOrDefaultExp(relativity.schwarzschildRadius, 2, ' m');
        
        // --- 4. Météo & UKF/IMU Debug ---
        $('air-temp-c').textContent = dataOrDefault(lastWeather.T_K - 273.15, 1, ' °C');
        $('pressure-hpa').textContent = dataOrDefault(lastWeather.P_hPa, 0, ' hPa');
        $('humidity-perc').textContent = dataOrDefault(lastWeather.H_perc, 1, ' %', 100);
        $('air-density').textContent = dataOrDefault(lastWeather.airDensity, 3, ' kg/m³');
        $('speed-of-sound-calc').textContent = dataOrDefault(lastWeather.speedOfSound, 2, ' m/s');
        
        $('no2-val').textContent = dataOrDefault(lastPollutants.no2, 0, ' µg/m³');
        $('pm25-val').textContent = dataOrDefault(lastPollutants.pm25, 0, ' µg/m³');
        $('pm10-val').textContent = dataOrDefault(lastPollutants.pm10, 0, ' µg/m³');
        $('o3-val').textContent = dataOrDefault(lastPollutants.o3, 0, ' µg/m³');

        // UKF/EKF
        $('lat-ekf').textContent = dataOrDefault(kLat, 6, ' °');
        $('lon-ekf').textContent = dataOrDefault(kLon, 6, ' °');
        $('alt-ekf').textContent = dataOrDefault(kAlt, 2, ' m');
        $('acc-gps').textContent = dataOrDefault(currentPosition.acc, 2, ' m');
        $('gps-accuracy-display').textContent = dataOrDefault(R_NOISE_OVERRIDE, 6, ' m');
        $('ukf-alt-sigma').textContent = dataOrDefault(ukfRNoise, 2, ' m'); // Sigma (Racine de R)
        $('ukf-r-noise').textContent = dataOrDefault(ukfRNoise * ukfRNoise, 4, ' m²'); // R
        
        // IMU (Attention: 'acceleration-x' est utilisé deux fois dans l'HTML)
        $('acceleration-x').textContent = dataOrDefault(imu.ax, 2, ' m/s²'); // Accélération Long.
        $('acceleration-y').textContent = dataOrDefault(imu.ay, 2, ' m/s²'); 
        $('acceleration-z').textContent = dataOrDefault(imu.az, 2, ' m/s²'); // Accel. Verticale
        $('mag-x').textContent = dataOrDefault(imu.mx, 2, ' µT');
        $('mag-y').textContent = dataOrDefault(imu.my, 2, ' µT');
        $('mag-z').textContent = dataOrDefault(imu.mz, 2, ' µT');
        
        $('inclinaison-pitch').textContent = dataOrDefault(imu.pitch, 1, ' °');
        $('roulis-roll').textContent = dataOrDefault(imu.roll, 1, ' °');
        
        // Mettre à jour le niveau à bulle
        if ($('bubble')) {
            const x = Math.min(40, Math.max(-40, imu.roll * 5));
            const y = Math.min(40, Math.max(-40, imu.pitch * 5));
            $('bubble').style.transform = `translate(${x}px, ${y}px)`;
        }

        // Vérification Météo
        if (now.getTime() - lastWeatherFetch > WEATHER_UPDATE_INTERVAL) {
             lastWeatherFetch = now.getTime(); 
             fetchWeather(kLat, kLon);
        }
        
    }, DOM_SLOW_UPDATE);
};

// --- Initialisation du Système (IIFE) ---
((window) => {
    document.addEventListener('DOMContentLoaded', async () => {
        // --- Vérification et Initialisation ---
        if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
            document.body.innerHTML = '<div style="color:red;padding:20px;">ERREUR CRITIQUE: Dépendances JS manquantes (math, leaflet, turf, suncalc).</div>';
            return;
        }

        initMap(kLat, kLon); 
        await syncNTP();
        setInterval(syncNTP, NTP_SYNC_INTERVAL);
        
        // Initialisation de l'UKF/IMU avec le delta T initial
        const initialDeltaT = (new Date().getTime() - startTimeMs) / 1000;
        updateSyntheticIMU(kSpd, currentMass, kAlt, initialDeltaT);

        // --- Configuration des Écouteurs d'Événements (CORRIGÉS) ---
        $('toggle-gps-btn').addEventListener('click', () => {
            isGpsRunning = !isGpsRunning;
            const options = $('freq-select').value === 'HIGH_FREQ' 
                ? { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
                : { enableHighAccuracy: false, maximumAge: 120000, timeout: 10000 };
                
            if (isGpsRunning) {
                $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
                $('toggle-gps-btn').style.backgroundColor = '#ffc107';
                startGpsWatch(options);
            } else {
                $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
                $('toggle-gps-btn').style.backgroundColor = '#28a745';
                if (watchId !== null) navigator.geolocation.clearWatch(watchId);
                watchId = null;
            }
        });
        
        $('freq-select').addEventListener('change', (e) => {
             const options = e.target.value === 'HIGH_FREQ' 
                ? { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
                : { enableHighAccuracy: false, maximumAge: 120000, timeout: 10000 };
             if (isGpsRunning) startGpsWatch(options);
        });

        $('emergency-stop-btn').addEventListener('click', () => {
            if (watchId !== null) navigator.geolocation.clearWatch(watchId);
            isGpsRunning = false;
            kSpd = 0; kAltDt = 0;
            $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: ACTIF 🔴';
            $('emergency-stop-btn').classList.add('active');
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS (REDÉMARRER)';
        });

        $('reset-dist-btn').addEventListener('click', () => { distM = 0; });
        $('reset-max-btn').addEventListener('click', () => { maxSpdMs = 0; });
        $('reset-all-btn').addEventListener('click', () => { location.reload(); });
        
        $('gps-accuracy-override').addEventListener('input', (e) => {
            R_NOISE_OVERRIDE = parseFloat(e.target.value) || 0.0;
            $('gps-accuracy-display').textContent = dataOrDefault(R_NOISE_OVERRIDE, 6, ' m');
        });

        $('environment-select').addEventListener('change', (e) => {
            currentEnvironment = e.target.value;
            const env = ENVIRONMENT_FACTORS[currentEnvironment];
            $('env-factor').textContent = `${env.DISPLAY} (x${env.R_MULT.toFixed(1)})`;
        });
        
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            G_ACC_CURRENT = calculateGravity(currentCelestialBody, kAlt, rotationRadius, angularVelocity, kLat * D2R);
            $('gravity-base').textContent = dataOrDefault(G_ACC_CURRENT, 4, ' m/s²');
        });
        
        const updateRotation = () => {
            rotationRadius = parseFloat($('rotation-radius').value) || 100;
            angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
            G_ACC_CURRENT = calculateGravity(currentCelestialBody, kAlt, rotationRadius, angularVelocity, kLat * D2R);
            $('gravity-base').textContent = dataOrDefault(G_ACC_CURRENT, 4, ' m/s²');
        };
        $('rotation-radius').addEventListener('input', updateRotation);
        $('angular-velocity').addEventListener('input', updateRotation);

        $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${distanceRatioMode ? 'ALT' : '1.000'})`;
        });

        $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1)'}`;
        });
        
        $('ukf-reactivity-mode').addEventListener('change', (e) => {
             // L'UKF_REACTIVITY est calculé à chaque étape pour un gain adaptatif plus professionnel.
             // Cette sélection pourrait ajuster le bruit de processus Q (mais ici on se contente de l'afficher)
             console.log(`Mode de réactivité UKF changé à ${e.target.value}`);
        });
        
        // Gérer le bouton du mode Jour/Nuit
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
            // Le filtre Leaflet n'est pas directement modifiable depuis le JS ici, il faut recharger la carte si on veut changer la couleur.
        });


        // --- Démarrage ---
        startDOMUpdateLoop();
        fetchWeather(kLat, kLon);
        
    });
})(window);
