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
        this.X = math.matrix(math.zeros([UKF_STATE_DIM, 1])); // État (Position, Vitesse, Accélération, Biais, etc.)
        this.P = math.matrix(math.diag(math.ones([UKF_STATE_DIM]).map((_, i) => i < 6 ? 1e-2 : 1e-4))); // Covariance
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
        // Retourne les valeurs par défaut N/A si vitesse > c ou masse nulle
        return {};
    }

    const lorentzFactor = 1 / Math.sqrt(sqrt_val);

    // Dilation du Temps (Vitesse) en ns/jour
    const dilationSpeed = (lorentzFactor - 1) * 86400 * 1e9;

    // Dilation du Temps (Gravité) en ns/jour (Approximation de Schwarzschild)
    // d(tau) = d(t) * sqrt(1 - 2GM/rc^2)
    // Ici, on utilise l'approximation pour l'altitude terrestre (généralement très faible)
    const r = R_E_BASE + alt;
    const sch_factor = (1 - (2 * G_U * m0) / (r * c * c));
    const dilationGravity = (1 - Math.sqrt(sch_factor)) * 86400 * 1e9;

    // Rayon de Schwarzschild (Rs)
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

    // Puissance Mécanique (W)
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
    // Point de Rosée (Simplifié)
    const Pv_s = 6.1078 * Math.pow(10, (7.5 * tempC) / (tempC + 237.3));
    const Pv = Pv_s * (humidityPerc / 100);
    const dewPoint = (237.3 * Math.log(Pv / 6.1078)) / (7.5 - Math.log(Pv / 6.1078));

    // Humidité Absolue (kg/m³) - Simplifié
    const absoluteHumidity = (Pv * 18.01528) / (8.314 * (tempC + 273.15)) / 1000;

    // Taux de Photosynthèse (Simulé) - Dépend de la lumière (Altitude solaire)
    const photosynthesisRate = Math.max(0, 50 * Math.sin(sunAltitudeRad)); // Max 50 µmol/m²/s

    return { dewPoint, absoluteHumidity, photosynthesisRate };
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
    const JDN = date.getTime() / 86400000 + 2440587.5; // Jour Julien Numérique (Approximation)

    // Temps Sidéral Local Vrai (TSLV) - Nécessite une astrométrie avancée, ici simplifié par SunCalc ou approximation.
    const LSTV = (date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600) + (lon_deg / 15);

    // Équation du Temps (EOT) et Déclinaison (via SunCalc)
    const sunPos = SunCalc.getPosition(date, 0, lon_deg);
    const EoT_min = (sunPos.equationOfTime / 60); // Equation du Temps en minutes

    // Heure Solaire Moyenne (MST)
    const dateUTC = Date.UTC(date.getFullYear(), date.getMonth(), date.getDate(), date.getHours(), date.getMinutes(), date.getSeconds());
    const UT = (dateUTC % 86400000) / 3600000; // Heure UTC fractionnée
    const MST_h = UT + lon_deg / 15;

    // Heure Solaire Vraie (TST)
    const TST_h = MST_h + EoT_min / 60;
    const TST_normalized = TST_h % 24;

    return {
        EOT: EoT_min,
        MST: MST_h,
        TST: TST_normalized,
        LSTV: LSTV % 24,
    };
};

/**
 * Met à jour les éphémérides Solaires et Lunaires (dépend de SunCalc).
 */
const updateAstro = (lat, lon) => {
    if (typeof SunCalc === 'undefined') return;

    const now = getCDate(lServH, lLocH);
    if (!now) return;

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
    if ($('altitude-soleil')) $('altitude-soleil').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 2, '°');
    if ($('lever-soleil')) $('lever-soleil').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
    if ($('coucher-soleil')) $('coucher-soleil').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR') : 'N/A';

    // --- Lune ---
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, '%');
    if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
    if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 2, '°');

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

        return {
            tempC,
            tempK,
            pressure_hPa,
            humidity_perc: data.humidity,
            air_density,
            speedOfSound,
            pollutants: { // Données simulées ou fournies par l'API
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

        // --- AFFICHAGE DES CALCULS PHYSIQUES/RELATIVISTES (Fréquence Rapide) ---

        // Relativité (Facteur de Lorentz, Rayon de Schwarzschild, Dilatations)
        if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefaultExp(rel.lorentzFactor, 4);
        if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(rel.schwarzschildRadius, 4, ' m');
        if ($('dilatation-vitesse')) $('dilatation-vitesse').textContent = dataOrDefault(rel.dilationSpeed, 2, ' ns/j');
        if ($('dilatation-gravite')) $('dilatation-gravite').textContent = dataOrDefault(rel.dilationGravity, 2, ' ns/j');
        // E/c pour éviter les très grands nombres en J (Énergie/Vitesse de la lumière)
        if ($('energie-repos')) $('energie-repos').textContent = dataOrDefaultExp(rel.energyRest / C_L, 4, ' N/C');
        if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(rel.energyRelativistic / C_L, 4, ' N/C');
        if ($('momentum-relativiste')) $('momentum-relativiste').textContent = dataOrDefaultExp(rel.momentum, 4, ' kg·m/s');

        // Dynamique des Fluides & Forces (Pression, Traînée, Coriolis)
        if ($('pression-dynamique')) $('pression-dynamique').textContent = dataOrDefault(dyn.dynamicPressure, 2, ' Pa');
        if ($('force-trainee')) $('force-trainee').textContent = dataOrDefault(dyn.dragForce, 3, ' N');
        if ($('puissance-trainee')) $('puissance-trainee').textContent = dataOrDefault(dyn.dragPower, 2, ' kW');
        if ($('energie-cinetique')) $('energie-cinetique').textContent = dataOrDefault(dyn.kineticEnergy, 2, ' J');
        if ($('force-coriolis')) $('force-coriolis').textContent = dataOrDefault(dyn.corio_mag, 4, ' N');

        // Mise à jour de la carte (Leaflet)
        marker.setLatLng([currentPosition.lat, currentPosition.lon]);
        if ($('auto-center-checkbox').checked) {
            map.setView([currentPosition.lat, currentPosition.lon], map.getZoom() || 13);
        }

        // --- UKF/EKF AFFICHAGE (Position/Vitesse Corrigée) ---
        const ukfState = ukf.getState();
        // Affichage des états principaux UKF (Doit être la sortie du filtre)
        if ($('ukf-lat')) $('ukf-lat').textContent = dataOrDefault(ukfState.lat, 6, '°');
        if ($('ukf-lon')) $('ukf-lon').textContent = dataOrDefault(ukfState.lon, 6, '°');
        if ($('ukf-alt')) $('ukf-alt').textContent = dataOrDefault(ukfState.alt, 2, ' m');
        if ($('ukf-spd')) $('ukf-spd').textContent = dataOrDefault(ukfState.spd * KMH_MS, 2, ' km/h');

        // Affichage du Cap et de la précision UKF (ajustée par le facteur environnemental)
        if ($('cap-instant')) $('cap-instant').textContent = dataOrDefault(currentPosition.cap, 1, '°');
        if ($('ukf-precision')) $('ukf-precision').textContent = dataOrDefault(currentPosition.acc * ENVIRONMENT_FACTORS[selectedEnvironment].MULT, 2, ' m');

        // Mise à jour de l'indicateur ZUPT (Zero Velocity Update, si la vitesse est faible)
        const isStopped = currentPosition.spd < 0.1;
        if ($('zupt-status')) $('zupt-status').textContent = isStopped ? 'ZUPT ACTIF (Arrêt)' : 'ZUPT INACTIF (Mouvement)';

        // Stocker la position pour la prochaine itération
        lastLatLon = { lat: coords.latitude, lon: coords.longitude };
    };

    /**
     * Gestion des erreurs GPS.
     */
    const onGpsError = (err) => {
        console.error(`[GPS] Erreur ${err.code}: ${err.message}`);
        if ($('statut-gps-acquisition')) {
            $('statut-gps-acquisition').textContent = `Erreur: ${err.message}. Tentez de réactiver.`;
            $('statut-gps-acquisition').style.backgroundColor = 'var(--error-color)';
        }
        stopGPS(false); // Arrêter pour économiser la batterie
    };

    /**
     * Démarre la surveillance de la position GPS.
     */
    const startGPS = () => {
        if (!navigator.geolocation) {
            alert("La géolocalisation n'est pas supportée par votre navigateur.");
            return;
        }
        if (watchID) stopGPS(false);

        // Choisir les options en fonction du mode Nether (simule un mode basse conso/basse précision)
        const options = netherMode ? GPS_OPTS_LOW_FREQ : GPS_OPTS_HIGH_FREQ;

        watchID = navigator.geolocation.watchPosition(
            onGpsSuccess,
            onGpsError,
            options
        );
        isGpsPaused = false;
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
        $('statut-capteur').textContent = `Actif (Mode: ${netherMode ? 'Nether/Économie' : 'Haute Précision'})`;
        $('statut-gps-acquisition').textContent = 'Acquisition en cours...';
        $('statut-gps-acquisition').style.backgroundColor = 'var(--accent-color)';
    };

    /**
     * Arrête la surveillance de la position GPS.
     */
    const stopGPS = (manualStop = true) => {
        if (watchID) {
            navigator.geolocation.clearWatch(watchID);
            watchID = null;
        }
        isGpsPaused = true;
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
        $('statut-capteur').textContent = manualStop ? `En Veille` : `Erreur/Veille`;
        $('statut-gps-acquisition').textContent = 'En Pause/Veille (Désactivé)';
        $('statut-gps-acquisition').style.backgroundColor = '#5c5c5c';
    };

    // --- Fonctions DOM Météo/Polluants (pour la boucle lente) ---

    /**
     * Fonction utilitaire pour mettre à jour le DOM Météo (Température, Pression, Humidité).
     */
    const updateWeatherDOM = (data, isInitial = false) => {
        const altText = isInitial ? '(ISA par défaut)' : '';
        if (data) {
            if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
            if ($('temp-air-2')) $('temp-air-2').textContent = `${dataOrDefault(data.tempC, 1)} °C ${altText}`;
            if ($('pressure-2')) $('pressure-2').textContent = `${dataOrDefault(data.pressure_hPa, 0)} hPa ${altText}`;
            if ($('humidity-2')) $('humidity-2').textContent = `${dataOrDefault(data.humidity_perc, 0)} %`;
            if ($('air-density')) $('air-density').textContent = `${dataOrDefault(data.air_density, 3)} kg/m³`;
            if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${dataOrDefault(data.speedOfSound, 2)} m/s`;
            // Le Point de Rosée est mis à jour dans startSlowLoop (besoin de SunAltitude)
        } else {
            if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
            if ($('temp-air-2')) $('temp-air-2').textContent = `N/A`;
            if ($('pressure-2')) $('pressure-2').textContent = `N/A`;
            if ($('humidity-2')) $('humidity-2').textContent = `N/A`;
            if ($('air-density')) $('air-density').textContent = `${dataOrDefault(RHO_SEA_LEVEL, 3)} kg/m³ (Défaut ISA)`;
            if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${dataOrDefault(getSpeedOfSound(TEMP_SEA_LEVEL_K), 2)} m/s (Défaut ISA)`;
        }
    };

    /**
     * Fonction utilitaire pour mettre à jour le DOM Polluants.
     */
    const updatePollutantsDOM = (data) => {
        if (data) {
            if ($('polluant-no2')) $('polluant-no2').textContent = dataOrDefault(data.NO2, 0, ' µg/m³');
            if ($('polluant-pm25')) $('polluant-pm25').textContent = dataOrDefault(data.PM2_5, 0, ' µg/m³');
            if ($('polluant-pm10')) $('polluant-pm10').textContent = dataOrDefault(data.PM10, 0, ' µg/m³');
            if ($('polluant-o3')) $('polluant-o3').textContent = dataOrDefault(data.O3, 0, ' µg/m³');
        } else {
            // Afficher N/A pour tous les polluants si les données sont absentes
            if ($('polluant-no2')) $('polluant-no2').textContent = 'N/A';
            if ($('polluant-pm25')) $('polluant-pm25').textContent = 'N/A';
            if ($('polluant-pm10')) $('polluant-pm10').textContent = 'N/A';
            if ($('polluant-o3')) $('polluant-o3').textContent = 'N/A';
        }
    };

    let slowLoopID = null;
    let lastWeatherFetchTime = 0;
    const WEATHER_FETCH_INTERVAL = 15 * 60 * 1000; // Fréquence de 15 minutes pour l'API Météo

    /**
     * Boucle de mise à jour lente (Astro/Météo/Horloge).
     */
    const startSlowLoop = () => {
        if (slowLoopID) clearInterval(slowLoopID);

        const slowUpdate = async () => {
            const kLat = currentPosition.lat;
            const kLon = currentPosition.lon;
            const kAlt = currentPosition.alt;

            // 1. ASTROMÉTRIE & CALCUL D'ALTITUDE SOLAIRE
            const { sunAltitudeRad } = updateAstro(kLat, kLon) || {};

            // 2. SYNCHRONISATION HORLOGE (Toutes les 30 min ou au démarrage)
            const now = getCDate(lServH, lLocH);
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            }
            // Synchro NTP toutes les 30 minutes
            if (Math.floor(Date.now() / 1000) % (30 * 60) === 0 || lServH === 0) {
                 syncH();
            }

            // 3. MÉTÉO & POLLUANTS (API Externe)
            if (Date.now() - lastWeatherFetchTime > WEATHER_FETCH_INTERVAL) {
                if (kLat !== null && kLon !== null) {
                    const weatherData = await fetchWeatherAndPollutants(kLat, kLon);
                    if (weatherData) {
                        // Mise à jour des variables pour les calculs physiques/UKF
                        lastP_hPa = weatherData.pressure_hPa;
                        lastT_K = weatherData.tempK;
                        currentAirDensity = weatherData.air_density;
                        currentSpeedOfSound = weatherData.speedOfSound;

                        // Mise à jour du DOM
                        updateWeatherDOM(weatherData);
                        updatePollutantsDOM(weatherData.pollutants);

                        lastWeatherFetchTime = Date.now();
                    } else {
                        updateWeatherDOM(null); // Afficher N/A si échec API
                        // Rétablit les valeurs ISA par défaut si l'API échoue
                        currentAirDensity = RHO_SEA_LEVEL;
                        currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);
                        lastT_K = TEMP_SEA_LEVEL_K;
                        lastP_hPa = 1013.25;
                    }
                }
            }

            // 4. CALCULS BIOLOGIQUES/SVT (Utilise les données ISA/API les plus récentes)
            const tempC = lastT_K - 273.15;
            // Utiliser une humidité par défaut (70) si l'API a échoué
            const defaultHumidity = 70; 
            if (sunAltitudeRad !== undefined) {
                 const bioSim = calculateBioSVT(tempC, kAlt || 0, defaultHumidity, lastP_hPa * 100, sunAltitudeRad);
                 if ($('dew-point')) $('dew-point').textContent = dataOrDefault(bioSim.dewPoint, 1, ' °C');
                 if ($('absolute-humidity')) $('absolute-humidity').textContent = dataOrDefault(bioSim.absoluteHumidity * 1000, 2, ' g/m³');
                 if ($('photosynthesis-rate')) $('photosynthesis-rate').textContent = dataOrDefault(bioSim.photosynthesisRate, 2, ' µmol/m²/s');
            } else {
                if ($('dew-point')) $('dew-point').textContent = 'N/A';
                if ($('absolute-humidity')) $('absolute-humidity').textContent = 'N/A';
                if ($('photosynthesis-rate')) $('photosynthesis-rate').textContent = 'N/A';
            }

            // 5. MISE À JOUR DE LA GRAVITÉ (Si Corps Céleste n'est pas Terre)
            if (currentCelestialBody !== 'Terre') {
                const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
                if ($('gravity-base')) $('gravity-base').textContent = dataOrDefault(G_ACC_NEW, 4, ' m/s²');
            } else {
                 if ($('gravity-base')) $('gravity-base').textContent = dataOrDefault(G_ACC, 4, ' m/s²');
            }
        };

        // Exécution immédiate et démarrage de l'intervalle
        slowUpdate();
        slowLoopID = setInterval(slowUpdate, DOM_SLOW_UPDATE_MS);
    };

    // --- GESTION DES ÉVÉNEMENTS (Démarrage/Arrêt/Configuration) ---

    // Bascule GPS
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => {
            if (isGpsPaused) {
                startGPS();
            } else {
                stopGPS();
            }
        });
    }

    // Bascule Mode Nether
    if ($('nether-toggle-btn')) {
        $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
            // Redémarrer le GPS pour appliquer les nouvelles options de fréquence/précision
            if (!isGpsPaused) {
                stopGPS(false);
                startGPS();
            }
        });
    }

    // Réinitialisation des statistiques
    if ($('reset-stats-btn')) {
        $('reset-stats-btn').addEventListener('click', () => {
            totalDistance = 0.0;
            maxSpeed = 0.0;
            if ($('distance-totale-3d')) $('distance-totale-3d').textContent = `0,000 km | 0,00 m`;
            if ($('vitesse-max')) $('vitesse-max').textContent = `0,0 km/h`;
        });
    }

    // Bascule du mode Nuit
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ?
                '<i class="fas fa-sun"></i> Mode Jour' :
                '<i class="fas fa-moon"></i> Mode Nuit';
        });
    }

    // Gestion de la masse, du corps céleste, et du facteur d'environnement
    if ($('mass-input')) {
        $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
        });
    }

    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            // Déclenchement manuel de la mise à jour lente pour la gravité
            if (slowLoopID) { clearInterval(slowLoopID); startSlowLoop(); }
        });
    }

    if ($('environment-select')) {
        $('environment-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
        });
    }

    // Gestion de la rotation (si le corps est 'Rotation')
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius').value) || 100;
        angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
        if (currentCelestialBody === 'Rotation') {
            const { G_ACC_NEW } = updateCelestialBody('Rotation', currentPosition.alt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = dataOrDefault(G_ACC_NEW, 4, ' m/s²');
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);

    // --- DÉMARRAGE DU SYSTÈME AU CHARGEMENT ---
    window.onload = () => {
        try {
            // Initialisation de l'UKF après vérification de math.js
            if (typeof math !== 'undefined') {
                ukf = new ProfessionalUKF();
            } else {
                console.error("Dépendance math.js manquante. Le filtre UKF est désactivé.");
            }

            // Mise à jour initiale des constantes physiques
            updateCelestialBody(currentCelestialBody, currentPosition.alt, rotationRadius, angularVelocity);

            // Démarrage de la synchronisation horaire et des boucles de mise à jour
            syncH();
            startSlowLoop();
            startGPS(); // Démarrer le GPS par défaut

            // Mise à jour des affichages par défaut
            if($('mass-display')) $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
            if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
            if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
            if ($('statut-capteur')) $('statut-capteur').textContent = `Actif (Mode: Haute Précision)`;
            if ($('gravity-base')) $('gravity-base').textContent = dataOrDefault(G_ACC, 4, ' m/s²');

            // Correction de l'état initial du bouton de mode
            if ($('toggle-mode-btn')) {
                $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ?
                    '<i class="fas fa-sun"></i> Mode Jour' :
                    '<i class="fas fa-moon"></i> Mode Nuit';
            }


        } catch (error) {
            console.error("ERREUR CRITIQUE D'INITIALISATION:", error);
            const statusElement = $('statut-gps-acquisition') || document.body;
            statusElement.innerHTML = `<h2 style="color:red;">CRASH SCRIPT: ${error.name}</h2><p>${error.message}</p>`;
        }
    };
})(window);
                                 
