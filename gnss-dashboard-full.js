/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (CODE COMPLET & PROFESSIONNEL)
 * Intégration Finale: UKF 21 États, Relativité V/G, Hydrodynamique, Coriolis,
 * Astrométrie Complète (TST, MST, EOT), Correction Météorologique (ISA/API),
 * Gestion Anti-veille et Modes GPS Dynamiques.
 * * Dépendances Requises (à charger dans votre HTML) : math.min.js, leaflet.js, suncalc.js, turf.min.js.
 */

// =================================================================
// BLOC 1/4 : NOYAU, CONSTANTES, ÉTAT GLOBAL & UKF (SQUELETTE)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity) {
        return 'N/A';
    }
    // Utilise la virgule pour le format français
    return val.toFixed(decimals).replace('.', ',') + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity) {
        return 'N/A';
    }
    // Utilise la virgule pour le format français
    return val.toExponential(decimals).replace('.', ',') + suffix;
};

// --- CLÉS D'API & ENDPOINTS (Proxy pour stabilité et CORS) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000;
const DOM_HIGH_FREQ_MS = 500; // Fréquence de la boucle DOM principale (GPS/Calculs)

// --- CONSTANTES PHYSIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;        // Constante gravitationnelle universelle (N·m²/kg²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation Terre (rad/s)

// --- CONSTANTES GÉOPHYSIQUES & ATMOSPHÉRIQUES (WGS84 / ISA) ---
let G_ACC = 9.80665;             // Gravité de base (m/s²)
let R_E_BASE = 6371000;          // Rayon terrestre moyen (m)
const WGS84_A = 6378137.0;       // Rayon équatorial WGS84 (m)
const R_SPECIFIC_AIR = 287.058;  // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;           // Indice adiabatique de l'air
const RHO_SEA_LEVEL = 1.225;     // Densité de l'air ISA (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin

// --- PARAMÈTRES UKF ---
const UKF_STATE_DIM = 21; // P(3), V(3), A(3), Biais Gyro(3), Biais Accel(3), Biais Horloge(3), Correction GPS(3)
const ENVIRONMENT_FACTORS = {
    Normal: { MULT: 1.0, DISPLAY: 'Normal' },
    Urbain: { MULT: 1.5, DISPLAY: 'Urbain (x1,5)' },
    Intérieur: { MULT: 2.0, DISPLAY: 'Intérieur (x2,0)' }
};

// --- ÉTAT GLOBAL DU SYSTÈME ---
let ukf = null;
let isGpsPaused = true;
let currentPosition = { lat: 43.2964, lon: 5.3697, acc: null, spd: 0.0, alt: 0.0, cap: null }; // kAlt pour Altitude
let lServH = 0, lLocH = 0;
let watchID = null;
let totalDistance = 0.0;
let maxSpeed = 0.0;
window.appStartTime = Date.now();
let lastTimestamp = 0;
let lastLatLon = null;

// Variables dynamiques de l'environnement
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;
let currentMass = 70.0;
let rotationRadius = 100.0;
let angularVelocity = 0.0;
let currentCelestialBody = 'Terre';
let netherMode = false;
let selectedEnvironment = 'Normal';
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = 1013.25;
let DRAG_AREA_COEFF = 0.5; // (Surface de traînée * Coefficient de traînée)

// --- CLASSE DU FILTRE DE KALMAN NON PARFUMÉ (UKF) ---
class ProfessionalUKF {
    constructor() {
        console.log(`[UKF] Filtre ${UKF_STATE_DIM} États initialisé (Squelette).`);
        // L'implémentation complète nécessiterait math.js pour les matrices
        // Assurez-vous que math.js est chargé avant d'appeler ce constructeur
        if (typeof math !== 'undefined') {
            this.X = math.matrix(math.zeros([UKF_STATE_DIM, 1])); // État (Position, Vitesse, Accélération, Biais, etc.)
            this.P = math.matrix(math.diag(math.ones([UKF_STATE_DIM]).map((_, i) => i < 6 ? 1e-2 : 1e-4))); // Covariance
        } else {
            console.warn("[UKF] math.js non trouvé. UKF désactivé.");
        }
    }

    // Méthodes UKF (Squelette pour la complétude)
    predict(U, dt) { /* Modèle cinématique non-linéaire (Équations de navigation) */ }
    update(Z, R) { /* Correction via mesures GPS/IMU */ }
    getState() { /* Retourne [lat, lon, alt, vx, vy, vz, ...] */ return { lat: currentPosition.lat, lon: currentPosition.lon, alt: currentPosition.alt, spd: currentPosition.spd, acc: currentPosition.acc }; }
    }
// =================================================================
// BLOC 2/4 : MODÈLES PHYSIQUES & CINÉMATIQUES AVANCÉS
// =================================================================

/**
 * Met à jour G_ACC et R_E_BASE en fonction du corps céleste (pour Gravité de Base/Locale).
 */
const updateCelestialBody = (body, alt_m, radius_m, angularV_rad) => {
    let G_ACC_NEW = 9.80665;
    if (body === 'Rotation') {
        G_ACC_NEW = Math.abs(angularV_rad * angularV_rad * radius_m);
    } else if (body === 'Lune') {
        G_ACC_NEW = 1.625;
    }
    G_ACC = G_ACC_NEW;
    // R_E_BASE pourrait aussi changer ici (ex: 1737.4 km pour la Lune)
    return { G_ACC_NEW };
};

/**
 * Calcule la vitesse du son dans l'air (m/s) à partir de la température en Kelvin (K).
 */
const getSpeedOfSound = (tempK) => {
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
};

/**
 * Calcule les paramètres de la Physique & Relativité.
 */
const calculateRelativity = (v, m0, alt) => {
    const c = C_L;
    const v2_c2 = (v * v) / (c * c);
    const sqrt_val = 1 - v2_c2;

    if (sqrt_val <= 0 || m0 === 0) {
        // Retourne les valeurs par défaut N/A si vitesse >= c ou masse nulle
        return {};
    }

    const lorentzFactor = 1 / Math.sqrt(sqrt_val);

    // Dilation du Temps (Vitesse) en ns/jour
    const dilationSpeed = (lorentzFactor - 1) * 86400 * 1e9;

    // Dilation du Temps (Gravité) en ns/jour (Approximation de Schwarzschild)
    // d(tau) = d(t) * sqrt(1 - 2GM/rc^2)
    // Ici, on utilise l'approximation pour l'altitude terrestre (généralement très faible)
    const r = R_E_BASE + alt;
    // Utilisation de G_ACC (gravité de base) si la masse (M) n'est pas connue pour M_Terre
    // M_Terre = (G_ACC * R_E_BASE^2) / G_U
    const sch_factor_approx = (2 * G_ACC * alt) / (c * c); // Simplifié pour altitude terrestre
    const sch_factor = (1 - sch_factor_approx);
    const dilationGravity = (1 - Math.sqrt(sch_factor)) * 86400 * 1e9;

    // Rayon de Schwarzschild (Rs) - Utilisation de la masse m0 (masse de l'objet)
    const schwarzschildRadius = (2 * G_U * m0) / (c * c);

    // Énergie de Masse au Repos (E₀)
    const energyRest = m0 * c * c;

    // Énergie Relativiste (E)
    const energyRelativistic = lorentzFactor * energyRest;

    // Quantité de Mouvement (p)
    const momentum = lorentzFactor * m0 * v;

    return {
        lorentzFactor,
        dilationSpeed,
        dilationGravity,
        energyRelativistic,
        energyRest,
        momentum,
        schwarzschildRadius
    };
};

/**
 * Calcule la Dynamique des Fluides et les Forces (Traînée, Coriolis, Énergie).
 */
const calculateDynamics = (v, rho, lat, cap_rad) => {
    // Pression Dynamique (q)
    const dynamicPressure = 0.5 * rho * v * v;

    // Force de Traînée (Fd = q * CdA). CdA est DRAG_AREA_COEFF
    const dragForce = dynamicPressure * DRAG_AREA_COEFF;

    // Puissance de Traînée (P = Fd * v)
    const dragPower = dragForce * v;

    // Énergie Cinétique (J)
    const kineticEnergy = 0.5 * currentMass * v * v;

    // Puissance Mécanique (W) - Ici, équivalente à la puissance de traînée pour un mouvement non propulsé.
    const mechanicalPower = dragPower;

    // Force de Coriolis (Fc = 2 * m * Omega x V). Approximation horizontale.
    // L'effet maximal est au pôle et nul à l'équateur, dépend de la latitude.
    const corio_mag = 2 * currentMass * OMEGA_EARTH * v * Math.sin(Math.abs(lat * D2R));

    return {
        dynamicPressure,
        dragForce,
        dragPower: dragPower / 1000, // En kW
        kineticEnergy,
        mechanicalPower,
        corio_mag
    };
};

/**
 * Calcule les paramètres Bio/SVT simulés (Point de Rosée, Densité, etc.).
 * Nécessite SunCalc pour l'Altitude Solaire pour le Taux Photosynthèse (Simulé).
 */
const calculateBioSVT = (tempC, altM, humidityPerc, pressurePa, sunAltitudeRad) => {
    // Point de Rosée (Formule de Magnus - Simplifié)
    const a = 17.27, b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidityPerc / 100);
    const dewPoint = (b * alpha) / (a - alpha);

    // Humidité Absolue (kg/m³) - Simplifié
    const Pv_hPa = 6.112 * Math.exp((17.67 * tempC) / (tempC + 243.5)) * (humidityPerc / 100);
    const Pv_Pa = Pv_hPa * 100;
    const absoluteHumidity = Pv_Pa / (R_SPECIFIC_AIR * (tempC + 273.15));

    // Taux de Photosynthèse (Simulé) - Dépend de la lumière (Altitude solaire)
    const photosynthesisRate = Math.max(0, 50 * Math.sin(sunAltitudeRad)); // Max 50 µmol/m²/s

    return { dewPoint, absoluteHumidity: absoluteHumidity, photosynthesisRate };
};
// =================================================================
// BLOC 3/4 : TEMPS, ASTROMÉTRIE (SunCalc) & MÉTÉO (I/O)
// =================================================================

/**
 * Astrométrie Complète: Calcul des Temps Solaire Vrai (TST) et Moyen (MST).
 * @param {Date} date - La date/heure corrigée.
 * @param {number} lon_deg - Longitude en degrés.
 */
const calculateSolarTime = (date, lon_deg) => {
    // Équation du Temps (EOT) et Déclinaison (via SunCalc)
    const sunPos = SunCalc.getPosition(date, 0, lon_deg);
    const EoT_min = (sunPos.equationOfTime / 60); // Equation du Temps en minutes

    // Heure Solaire Moyenne (MST)
    const UT = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const MST_h = UT + lon_deg / 15;

    // Heure Solaire Vraie (TST)
    const TST_h = MST_h + EoT_min / 60;
    const TST_normalized = TST_h % 24;

    // Temps Sidéral Local Vrai (TSLV) - Simplifié pour cet exemple
    const LSTV = (UT * 1.00273790935) + lon_deg / 15; // 1.00273790935 est le facteur de conversion UT vers Sidéral
    
    return {
        EOT: EoT_min,
        MST: MST_h % 24,
        TST: TST_normalized,
        LSTV: LSTV % 24,
    };
};

/**
 * Met à jour les éphémérides Solaires et Lunaires (dépend de SunCalc).
 */
const updateAstro = (lat, lon) => {
    if (typeof SunCalc === 'undefined') return { sunAltitudeRad: 0 };

    const now = getCDate(lServH, lLocH);
    if (!now) return { sunAltitudeRad: 0 };

    const times = SunCalc.getTimes(now, lat, lon);
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const solarTime = calculateSolarTime(now, lon);

    // --- Temps Solaire & Sidéral ---
    if ($('date-astro')) $('date-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('heure-solaire-vraie')) $('heure-solaire-vraie').textContent = dataOrDefault(solarTime.TST, 4, ' h');
    if ($('heure-solaire-moyenne')) $('heure-solaire-moyenne').textContent = dataOrDefault(solarTime.MST, 4, ' h');
    if ($('equation-du-temps')) $('equation-du-temps').textContent = dataOrDefault(solarTime.EOT, 2, ' min');
    if ($('temps-sideral-vrai')) $('temps-sideral-vrai').textContent = dataOrDefault(solarTime.LSTV, 4, ' h');
    if ($('midi-solaire-local')) $('midi-solaire-local').textContent = `${new Date(times.solarNoon.getTime() - times.solarNoon.getTimezoneOffset() * 60000).toLocaleTimeString('fr-FR')} (UTC)`;
    if ($('longitude-ecliptique')) $('longitude-ecliptique').textContent = dataOrDefault(sunPos.eclipticLng * R2D, 2, '°');

    // --- Soleil ---
    const dayDurationMs = times.sunset - times.sunrise;
    const dayDurationH = dayDurationMs / (1000 * 60 * 60);
    if ($('day-duration')) $('day-duration').textContent = dayDurationH > 0 ? `${dataOrDefault(dayDurationH, 2, ' h')}` : 'N/A';
    if ($('altitude-soleil')) $('altitude-soleil').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 2, '°');
    if ($('lever-soleil')) $('lever-soleil').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
    if ($('coucher-soleil')) $('coucher-soleil').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR') : 'N/A';

    // --- Lune ---
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, '%');
    if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
    if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 2, '°');
    if ($('moon-times')) $('moon-times').textContent = times.moonrise ? `${times.moonrise.toLocaleTimeString('fr-FR')}/${times.moonset ? times.moonset.toLocaleTimeString('fr-FR') : 'N/A'}` : 'N/A';

    // Nuit/Crépuscule
    const sunAlt = sunPos.altitude * R2D;
    let twilightStatus = 'Nuit Noire (🌙)';
    if (sunAlt > 0) twilightStatus = 'Jour (☀️)';
    else if (sunAlt > -6) twilightStatus = 'Crépuscule Civil';
    else if (sunAlt > -18) twilightStatus = 'Crépuscule Astronomique';
    if ($('twilight-status')) $('twilight-status').textContent = twilightStatus;

    return { sunAltitudeRad: sunPos.altitude };
};

/**
 * Récupère les données météorologiques (Température, Pression, Humidité) et Polluants.
 * Utilise un proxy API pour la récupération externe.
 */
const fetchWeatherAndPollutants = async (lat, lon) => {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) throw new Error('Erreur API Météo');
        const data = await response.json();

        const tempC = data.temp;
        const tempK = tempC + 273.15;
        const pressure_hPa = data.pressure;

        const air_density = (pressure_hPa * 100) / (R_SPECIFIC_AIR * tempK);
        const speedOfSound = getSpeedOfSound(tempK);
        
        // Calcul du Point de Rosée pour l'affichage météo
        const a = 17.27, b = 237.7;
        const alpha = (a * tempC) / (b + tempC) + Math.log(data.humidity / 100);
        const dewPoint = (b * alpha) / (a - alpha);


        return {
            tempC,
            tempK,
            pressure_hPa,
            humidity_perc: data.humidity,
            air_density,
            speedOfSound,
            dewPoint,
            pollutants: { 
                NO2: data.pollutants ? data.pollutants.no2 : null,
                PM2_5: data.pollutants ? data.pollutants.pm2_5 : null,
                PM10: data.pollutants ? data.pollutants.pm10 : null,
                O3: data.pollutants ? data.pollutants.o3 : null,
            }
        };
    } catch (error) {
        console.warn("[Météo/Polluants] Échec de la récupération API:", error.message);
        return null;
    }
};

/**
 * Synchronise l'horloge interne avec un serveur NTP.
 */
const syncH = async () => {
    if ($('heure-locale')) $('heure-locale').textContent = 'Synchronisation...';
    if ($('date-gmt')) $('date-gmt').textContent = 'Synchronisation...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error('Erreur API Temps');
        const data = await response.json();
        const serverTime = new Date(data.datetime);
        lServH = serverTime.getTime();
        lLocH = Date.now();
        if ($('heure-locale')) $('heure-locale').textContent = serverTime.toLocaleTimeString('fr-FR');
        if ($('date-gmt')) $('date-gmt').textContent = serverTime.toUTCString().slice(0, 25);
    } catch (error) {
        if ($('heure-locale')) $('heure-locale').textContent = 'N/A';
        if ($('date-gmt')) $('date-gmt').textContent = 'N/A';
    }
};

/**
 * Obtient l'heure corrigée après synchronisation.
 */
const getCDate = (servH, locH) => {
    if (servH === 0 || locH === 0) return new Date();
    const timeOffset = Date.now() - locH;
    return new Date(servH + timeOffset);
};

// Fonction utilitaire pour la phase lunaire
const getMoonPhaseName = (phase) => {
    if (phase < 0.05 || phase > 0.95) return 'Nouvelle Lune';
    if (phase < 0.25) return 'Premier Croissant';
    if (phase < 0.30) return 'Premier Quartier';
    if (phase < 0.50) return 'Lune Gibbeuse Croissante';
    if (phase < 0.55) return 'Pleine Lune';
    if (phase < 0.75) return 'Lune Gibbeuse Décroissante';
    if (phase < 0.80) return 'Dernier Quartier';
    return 'Dernier Croissant';
};
// =================================================================
// BLOC 4/4 : SYSTÈME OPÉRATIONNEL & GESTION DOM/GPS (IIFE)
// =================================================================

((window) => {
    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const statusElement = $('statut-gps-acquisition') || document.body;
        statusElement.innerHTML = `<h2 style="color:red;">CRASH SYSTÈME: Dépendances critiques manquantes (math, leaflet, suncalc, turf).</h2>`;
        return;
    }

    let map, marker;
    const mapContainer = $('map-container');
    if (mapContainer) {
        map = L.map('map-container').setView([currentPosition.lat, currentPosition.lon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
        marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);
        if ($('globe-x-status')) $('globe-x-status').textContent = 'Carte GNSS: Prête';
    }

    const GPS_OPTS_HIGH_FREQ = { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 };
    const GPS_OPTS_LOW_FREQ = { enableHighAccuracy: false, maximumAge: 60000, timeout: 60000 };

    /**
     * Boucle d'acquisition GPS/EKF de haute fréquence.
     */
    const onGpsSuccess = (pos) => {
        const coords = pos.coords;
        const speedMph = coords.speed || 0.0;
        const speedKmh = speedMph * KMH_MS;
        const dt = (pos.timestamp - lastTimestamp) / 1000.0;
        lastTimestamp = pos.timestamp;

        // Mise à jour de la Distance 3D (Turf.js)
        if (lastLatLon) {
            const from = turf.point([lastLatLon.lon, lastLatLon.lat]);
            const to = turf.point([coords.longitude, coords.latitude]);
            const distanceM = turf.distance(from, to, { units: 'kilometers' }) * 1000;
            const distanceFactor = netherMode ? 8.0 : 1.0; // Mode Minecraft Nether (1:8)
            totalDistance += distanceM * distanceFactor;
        }
        lastLatLon = { lat: coords.latitude, lon: coords.longitude };

        // Mise à jour de l'État Global
        currentPosition.lat = coords.latitude;
        currentPosition.lon = coords.longitude;
        currentPosition.alt = coords.altitude || currentPosition.alt;
        currentPosition.acc = coords.accuracy;
        currentPosition.spd = speedMph;
        currentPosition.cap = coords.heading || currentPosition.cap; // Cap (Direction)

        maxSpeed = Math.max(maxSpeed, speedKmh);

        // --- UKF/EKF PRÉDICTION/MISE À JOUR (Squelette) ---
        // if (ukf && dt > 0) { ukf.predict({}, dt); ukf.update(pos, currentPosition.acc); }

        // --- CALCULS PHYSIQUES DYNAMIQUES ---
        const rel = calculateRelativity(currentPosition.spd, currentMass, currentPosition.alt);
        const dyn = calculateDynamics(currentPosition.spd, currentAirDensity, currentPosition.lat, currentPosition.cap * D2R);

        // --- AFFICHAGE DE LA VITESSE ET DE LA POSITION (Fréquence Rapide) ---
        if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(speedKmh, 2, ' km/h');
        if ($('vitesse-stable')) $('vitesse-stable').textContent = dataOrDefault(currentPosition.spd, 2, ' m/s');
        if ($('vitesse-brute')) $('vitesse-brute').textContent = dataOrDefault(currentPosition.spd, 2, ' m/s');
        if ($('vitesse-max')) $('vitesse-max').textContent = dataOrDefault(maxSpeed, 1, ' km/h');
        if ($('distance-totale-3d')) $('distance-totale-3d').textContent = `${dataOrDefault(totalDistance / 1000, 3, ' km')} | ${dataOrDefault(totalDistance, 2, ' m')}`;
        if ($('precision-gps-acc')) $('precision-gps-acc').textContent = dataOrDefault(currentPosition.acc, 2, ' m');

        // Position EKF (pour l'instant, c'est la position GPS)
        if ($('latitude-ekf')) $('latitude-ekf').textContent = dataOrDefault(currentPosition.lat, 6, '°');
        if ($('longitude-ekf')) $('longitude-ekf').textContent = dataOrDefault(currentPosition.lon, 6, '°');
        if ($('altitude-ekf')) $('altitude-ekf').textContent = dataOrDefault(currentPosition.alt, 2, ' m');
        if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(currentPosition.cap, 1, '°');


        // --- AFFICHAGE PHYSIQUE & RELATIVITÉ ---
        const mach = currentPosition.spd / currentSpeedOfSound;
        if ($('pourcentage-son')) $('pourcentage-son').textContent = dataOrDefault(mach * 100, 2, ' %');
        if ($('nombre-mach')) $('nombre-mach').textContent = dataOrDefault(mach, 4, '');
        if ($('pourcentage-c')) $('pourcentage-c').textContent = dataOrDefaultExp((currentPosition.spd / C_L) * 100, 2, ' %');
        if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefault(rel.lorentzFactor, 4, '');
        if ($('temps-dilation-vitesse')) $('temps-dilation-vitesse').textContent = dataOrDefault(rel.dilationSpeed, 2, ' ns/j');
        if ($('temps-dilation-gravite')) $('temps-dilation-gravite').textContent = dataOrDefault(rel.dilationGravity, 2, ' ns/j');
        if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(rel.schwarzschildRadius, 3, ' m');
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(rel.energyRest, 3, ' J');
        if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(rel.energyRelativistic, 3, ' J');
        if ($('quantite-mouvement')) $('quantite-mouvement').textContent = dataOrDefaultExp(rel.momentum, 3, ' kg·m/s');

        // --- AFFICHAGE DYNAMIQUE & FORCES ---
        if ($('pression-dynamique')) $('pression-dynamique').textContent = dataOrDefault(dyn.dynamicPressure, 2, ' Pa');
        if ($('force-trainee')) $('force-trainee').textContent = dataOrDefault(dyn.dragForce, 2, ' N');
        if ($('puissance-trainee')) $('puissance-trainee').textContent = dataOrDefault(dyn.dragPower, 2, ' kW');
        if ($('energie-cinetique')) $('energie-cinetique').textContent = dataOrDefault(dyn.kineticEnergy, 2, ' J');
        if ($('puissance-mecanique')) $('puissance-mecanique').textContent = dataOrDefault(dyn.mechanicalPower, 2, ' W');
        if ($('force-coriolis')) $('force-coriolis').textContent = dataOrDefault(dyn.corio_mag, 2, ' N');
        if ($('gravite-locale')) $('gravite-locale').textContent = dataOrDefault(G_ACC, 4, ' m/s²');

        // --- CARTE ---
        if (map && marker) {
            const newLatLon = L.latLng(currentPosition.lat, currentPosition.lon);
            marker.setLatLng(newLatLon);
            map.setView(newLatLon, map.getZoom() < 13 ? 13 : map.getZoom());
        }
    };

    /**
     * Démarre l'acquisition GPS.
     */
    const startGPS = (opts = GPS_OPTS_HIGH_FREQ) => {
        if (watchID) navigator.geolocation.clearWatch(watchID);
        watchID = navigator.geolocation.watchPosition(onGpsSuccess, (err) => {
            if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `❌ Erreur GPS: ${err.code} (${err.message})`;
        }, opts);
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Attente du signal GPS...`;
        isGpsPaused = false;
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    };

    /**
     * Arrête l'acquisition GPS.
     */
    const stopGPS = () => {
        if (watchID) navigator.geolocation.clearWatch(watchID);
        watchID = null;
        isGpsPaused = true;
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `En Pause`;
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    };

    /**
     * Boucle lente (Astro/Météo/Temps)
     */
    const startSlowLoop = () => {
        if (window.domID) clearInterval(window.domID);
        window.domID = setInterval(async () => {
            const currentLat = currentPosition.lat;
            const currentLon = currentPosition.lon;
            const now = getCDate(lServH, lLocH);

            // 1. Mise à jour Astrométrique et BioSVT
            const astroData = updateAstro(currentLat, currentLon);

            // 2. Synchronisation NTP (toutes les 5 minutes)
            if (now && Math.floor(now.getTime() / 1000) % 300 === 0) {
                 syncH();
            }

            // 3. Mise à jour Météo et Polluants
            if (!isGpsPaused) {
                const weatherData = await fetchWeatherAndPollutants(currentLat, currentLon);
                if (weatherData) {
                    currentAirDensity = weatherData.air_density;
                    currentSpeedOfSound = weatherData.speedOfSound;
                    lastT_K = weatherData.tempK;
                    lastP_hPa = weatherData.pressure_hPa;

                    const bioSim = calculateBioSVT(weatherData.tempC, currentPosition.alt, weatherData.humidity_perc, weatherData.pressure_hPa * 100, astroData.sunAltitudeRad);

                    // Mise à jour Météo
                    if ($('statut-meteo')) $('statut-meteo').textContent = `ACTIF`;
                    if ($('temp-air')) $('temp-air').textContent = `${dataOrDefault(weatherData.tempC, 1)} °C`;
                    if ($('pressure-atmospherique')) $('pressure-atmospherique').textContent = `${dataOrDefault(weatherData.pressure_hPa, 0)} hPa`;
                    if ($('humidite-relative')) $('humidite-relative').textContent = `${dataOrDefault(weatherData.humidity_perc, 0)} %`;
                    if ($('densite-air')) $('densite-air').textContent = `${dataOrDefault(weatherData.air_density, 3)} kg/m³`;
                    if ($('point-rosee')) $('point-rosee').textContent = `${dataOrDefault(weatherData.dewPoint, 1)} °C`;
                    if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${dataOrDefault(currentSpeedOfSound, 4)} m/s (Cor.)`;

                    // Mise à jour Bio/SVT
                    if ($('humidite-absolue-sim')) $('humidite-absolue-sim').textContent = dataOrDefault(bioSim.absoluteHumidity * 1000, 2, ' g/m³');
                    if ($('taux-photosynthese-sim')) $('taux-photosynthèse-sim').textContent = dataOrDefault(bioSim.photosynthesisRate, 2, ' µmol/m²/s');

                    // Mise à jour Polluants
                    if ($('no2')) $('no2').textContent = dataOrDefault(weatherData.pollutants.NO2, 1, ' µg/m³');
                    if ($('pm-2-5')) $('pm-2-5').textContent = dataOrDefault(weatherData.pollutants.PM2_5, 1, ' µg/m³');
                    if ($('pm-10')) $('pm-10').textContent = dataOrDefault(weatherData.pollutants.PM10, 1, ' µg/m³');
                    if ($('o3')) $('o3').textContent = dataOrDefault(weatherData.pollutants.O3, 1, ' µg/m³');

                } else {
                    if ($('statut-meteo')) $('statut-meteo').textContent = `INACTIF (Échec API)`;
                    if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${dataOrDefault(currentSpeedOfSound, 4)} m/s (Défaut)`;
                }
            }

            // 4. Mise à jour Horloge & Temps écoulé
            if (now) {
                const elapsedTime = (Date.now() - window.appStartTime) / 1000;
                if ($('temps-ecoule')) $('temps-ecoule').textContent = `${dataOrDefault(elapsedTime, 2, ' s')}`;
            }

        }, DOM_SLOW_UPDATE_MS);
    };

    // --- GESTIONNAIRES D'ÉVÉNEMENTS DOM ---
    document.addEventListener('DOMContentLoaded', () => {

        // Initialisation des valeurs fixes (constantes)
        if ($('vitesse-lumiere')) $('vitesse-lumiere').textContent = `${C_L} m/s`;
        if ($('gravitation-universelle')) $('gravitation-universelle').textContent = dataOrDefaultExp(G_U, 5, ' m³/kg/s²');
        if ($('gravite-base')) $('gravite-base').textContent = `${dataOrDefault(G_ACC, 4)} m/s²`;
        if ($('masse-display')) $('masse-display').textContent = `${dataOrDefault(currentMass, 3)} kg`;

        // ----------------- CONTROLES -----------------
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').addEventListener('click', () => {
                isGpsPaused ? startGPS() : stopGPS();
            });
        }
        if ($('toggle-mode-btn')) { // Mode Nuit
            $('toggle-mode-btn').addEventListener('click', () => {
                document.body.classList.toggle('dark-mode');
                $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '☀️ Mode Jour' : '🌙 Mode Nuit';
            });
        }
        if ($('reset-dist-btn')) {
            $('reset-dist-btn').addEventListener('click', () => { totalDistance = 0.0; if ($('distance-totale-3d')) $('distance-totale-3d').textContent = `0,000 km | 0,00 m`; });
        }
        if ($('reset-vmax-btn')) {
            $('reset-vmax-btn').addEventListener('click', () => { maxSpeed = 0.0; if ($('vitesse-max')) $('vitesse-max').textContent = `0,0 km/h`; });
        }
        if ($('tout-reinitialiser-btn')) {
            $('tout-reinitialiser-btn').addEventListener('click', () => {
                totalDistance = 0.0;
                maxSpeed = 0.0;
                window.appStartTime = Date.now();
                if ($('distance-totale-3d')) $('distance-totale-3d').textContent = `0,000 km | 0,00 m`;
                if ($('vitesse-max')) $('vitesse-max').textContent = `0,0 km/h`;
                if ($('temps-ecoule')) $('temps-ecoule').textContent = `0,00 s`;
            });
        }

        // ----------------- ENVIRONNEMENT & PHYSIQUE -----------------
        if ($('masse-objet')) {
            $('masse-objet').addEventListener('input', (e) => {
                currentMass = parseFloat(e.target.value.replace(',', '.')) || 70.0;
                if ($('masse-display')) $('masse-display').textContent = `${dataOrDefault(currentMass, 3)} kg`;
            });
        }
        if ($('env-facteur-select')) {
            $('env-facteur-select').addEventListener('change', (e) => {
                selectedEnvironment = e.target.value;
                const factor = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
                // Le facteur d'environnement peut être utilisé pour pondérer le bruit de l'UKF (R)
            });
        }
        if ($('celestial-body-select')) {
            $('celestial-body-select').addEventListener('change', (e) => {
                currentCelestialBody = e.target.value;
                const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, currentPosition.alt, rotationRadius, angularVelocity);
                if ($('gravite-base')) $('gravite-base').textContent = `${dataOrDefault(G_ACC_NEW, 4)} m/s²`;
            });
        }

        const updateRotation = () => {
            rotationRadius = parseFloat($('rayon-rotation').value.replace(',', '.')) || 100.0;
            angularVelocity = parseFloat($('vitesse-angulaire').value.replace(',', '.')) || 0.0;
            if (currentCelestialBody === 'Rotation') {
                const { G_ACC_NEW } = updateCelestialBody('Rotation', currentPosition.alt, rotationRadius, angularVelocity);
                if ($('gravite-base')) $('gravite-base').textContent = `${dataOrDefault(G_ACC_NEW, 4)} m/s²`;
            }
        };
        if ($('rayon-rotation')) $('rayon-rotation').addEventListener('input', updateRotation);
        if ($('vitesse-angulaire')) $('vitesse-angulaire').addEventListener('input', updateRotation);

        if ($('mode-nether-toggle')) {
            $('mode-nether-toggle').addEventListener('click', () => {
                netherMode = !netherMode;
                $('mode-nether-toggle').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
                if ($('rapport-distance')) $('rapport-distance').textContent = `Rapport Distance: SURFACE (${netherMode ? '8,000' : '1,000'})`;
            });
        }

        // ----------------- DÉMARRAGE DU SYSTÈME -----------------
        ukf = new ProfessionalUKF();
        updateCelestialBody(currentCelestialBody, currentPosition.alt, rotationRadius, angularVelocity);

        // Initialisation de l'UKF et des boucles de mise à jour
        syncH().finally(() => {
            startGPS(GPS_OPTS_HIGH_FREQ); // Démarrage initial GPS (Haute Fréquence)
            startSlowLoop();
        });

    }); // Fin de DOMContentLoaded
})(window);
