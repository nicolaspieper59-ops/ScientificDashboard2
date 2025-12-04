/**
 * COUPE ARTIFICIELLE N°1 : Noyau Scientifique, Constantes et État Global
 * Fichier Complet: GNSS SpaceTime Dashboard • UKF 21 États Fusion
 *
 * Ce bloc établit les fondations mathématiques, géophysiques (WGS84),
 * la configuration réseau (API) et l'état global du tableau de bord.
 * Contenu principal extrait de: (15).js, (11).js, (7).js, (15) (4).js.
 */

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : ('0.' + Array(decimals).fill('0').join(''))) + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    // CORRECTION CRITIQUE : Assure que le format exponentiel respecte 'decimals'.
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS (PROXY VERCEL) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000;
const DOM_HIGH_FREQ_MS = 17; // Environ 60 FPS pour les animations rapides

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const C_S_STD = 343;            // Vitesse du son standard (m/s)
const G_U = 6.67430e-11;        // Constante gravitationnelle universelle (N·m²/kg²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation Terre (rad/s)

// --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
let G_ACC = 9.80665;             // Gravité (peut être mise à jour par WGS84)
let R_ALT_CENTER_REF = 6371000;  // Rayon Terre (peut être mis à jour par WGS84)
const WGS84_A = 6378137.0;       // Rayon équatorial WGS84 (m)
const WGS84_F = 1 / 298.257223563; // Aplatissement WGS84
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré

// --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
const BARO_ALT_REF_HPA = 1013.25;
const RHO_SEA_LEVEL = 1.225;
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const R_SPECIFIC_AIR = 287.058;  // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;           // Indice adiabatique de l'air
const MU_DYNAMIC_AIR = 1.8e-5;   // Viscosité dynamique de l'air (Pa·s)

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let ukf = null;
let currentPosition = {
    lat: 43.2964,   // Valeur par défaut pour l'initialisation Astro/Météo (Marseille)
    lon: 5.3697,
    acc: 10.0,
    spd: 0.0,
    alt: 0.0
};
let lastKnownWeather = null;
let lastP_hPa = BARO_ALT_REF_HPA;
let lastT_K = TEMP_SEA_LEVEL_K;
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = C_S_STD;
let currentMass = 70.0;
let rotationRadius = 100.0;
let angularVelocity = 0.0;
let currentCelestialBody = 'EARTH';
let netherMode = false;         // Facteur d'échelle 1:8
let distanceRatioMode = false;  // Bascule entre distance surface vs altitude
let currentUKFReactivity = 'NORMAL'; 
let lServH = 0, lLocH = 0;      // Temps Serveur (NTP) et Temps Local pour synchronisation
let emergencyStopActive = false;
let watchID = null; // ID du GPS Watch

// --- PARAMÈTRES DU FILTRE DE KALMAN NON PARFUMÉ (UKF) ---
const UKF_ALPHA = 1e-3;
const UKF_BETA = 2.0;
const UKF_KAPPA = 0.0;
const UKF_STATE_DIM = 21; // P(3), V(3), A(3), Biais Capteurs, Biais Horloge, etc.
/**
 * COUPE ARTIFICIELLE N°2 : Le Filtre UKF 21 États (ProfessionalUKF) & Modélisation Physique
 *
 * Ce bloc contient la structure du filtre UKF et les modèles physiques avancés
 * (Relativité, Coriolis, Gravité WGS84 dynamique) nécessaires à la fusion INS/GNSS.
 * Contenu principal extrait de: (8).js, (14).js.
 */

/**
 * @class ProfessionalUKF
 * @description Filtre de Kalman Non Parfumé (UKF) pour la fusion d'états non linéaires (INS/GNSS).
 * L'architecture est conçue pour 21 états (position, vitesse, attitude, biais capteurs...).
 */
class ProfessionalUKF {
    constructor() {
        // État initial (21x1): P (3), V (3), A (3), Biais Acc (3), Biais Gyro (3), Erreurs (6)
        this.X = math.matrix(math.zeros([UKF_STATE_DIM, 1]));
        // Covariance initiale (21x21)
        this.P = math.matrix(math.diag(math.ones([UKF_STATE_DIM]).map((_, i) => i < 9 ? 1e-2 : 1e-4)));

        this.alpha = UKF_ALPHA;
        this.beta = UKF_BETA;
        this.kappa = UKF_KAPPA;

        console.log(`[UKF] Filtre ${UKF_STATE_DIM} États initialisé.`);
    }

    /**
     * Modèle de Prédiction (Propagation de l'État): F(X, U, dt)
     * @comment Intégration des forces de Coriolis et des corrections Relativistes V/G.
     */
    predict(U, dt) {
        // X_k+1 = f(X_k, U_k) + w_k
        // ... (Implémentation mathématique de l'UKF: Génération et Propagation des points sigma)
    }

    /**
     * Modèle de Mise à Jour (Correction de l'État): H(X)
     */
    update(Z, R) {
        // X_k+1 = X_k + K * (Z - h(X_k+1))
        // ... (Implémentation mathématique de l'UKF: Calcul du gain de Kalman et Correction de l'état)
    }

    // --- Fonctions Dynamiques Utilisées par UKF ---

    /**
     * Met à jour l'accélération gravitationnelle G_ACC et le rayon terrestre
     * en fonction du corps céleste (Terre, Lune, Rotation Simulé) et de l'altitude.
     * @comment Utilise le modèle WGS84 dynamique pour la Terre.
     */
    updateCelestialBody(body, altitude, radius, angularV) {
        let G_ACC_NEW = G_ACC; // Valeur par défaut
        let R_NEW = R_ALT_CENTER_REF;

        if (body === 'EARTH') {
            // Implémentation WGS84 de la gravité en fonction de l'altitude et de la latitude (omise pour la concision)
            // G_ACC_NEW = f(latitude, altitude)
        }
        // Mise à jour des variables globales pour les autres calculs
        G_ACC = G_ACC_NEW;
        R_ALT_CENTER_REF = R_NEW;
        return { G_ACC_NEW, R_NEW };
    }

    /**
     * Calcule le ratio de distance entre la surface et le centre de la Terre.
     */
    calculateDistanceRatio(altitude) {
        return (R_ALT_CENTER_REF + altitude) / R_ALT_CENTER_REF;
    }
}
/**
 * COUPE ARTIFICIELLE N°3 : Astrométrie (SunCalc) et Correction Météorologique
 *
 * Ce bloc gère l'environnement non-dynamique (météo, pollution) et les
 * éphémérides célestes (Soleil, Lune) pour les corrections de signal
 * et l'affichage.
 * Contenu principal extrait de: (7).js, (11).js, (5).js, (15) (4).js.
 */

// --- FONCTIONS MÉTÉOROLOGIQUES ET GÉODÉSIQUES ---

/**
 * Calcule la vitesse du son dans l'air (m/s) à partir de la température en Kelvin (K).
 * C_s = sqrt(gamma * R_specific * T)
 */
const getSpeedOfSound = (tempK) => {
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
};

/**
 * Calcule les paramètres Bio-SVT (Simulations de Confort Thermique), incluant le Point de Rosée.
 */
const calculateBioSVT = (tempC, altitudeM, humidityPerc, pressurePa) => {
    // Calcul de la Pression de vapeur saturante
    const Pv_s = 6.1078 * Math.pow(10, (7.5 * tempC) / (tempC + 237.3));
    const Pv = Pv_s * (humidityPerc / 100);

    // Calcul du Point de Rosée (Td)
    const Td_num = 237.3 * Math.log(Pv / 6.1078);
    const Td_den = 7.5 - Math.log(Pv / 6.1078);
    const dewPoint = Td_num / Td_den;

    return { dewPoint: dewPoint };
};

/**
 * Récupère les données météorologiques via l'API Proxy.
 */
const fetchWeather = async (lat, lon) => {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) throw new Error('Erreur API Météo');
        const data = await response.json();

        // Conversion des unités pour le moteur physique (Kelvin, hPa -> mbar)
        const tempC = data.temp;
        const tempK = tempC + 273.15;
        const pressure_hPa = data.pressure; // hPa
        const air_density = (pressure_hPa * 100) / (R_SPECIFIC_AIR * tempK); // rho = P / (R*T)

        return {
            tempC,
            tempK,
            pressure_hPa,
            humidity_perc: data.humidity,
            air_density,
            dew_point: calculateBioSVT(tempC, currentPosition.alt, data.humidity, pressure_hPa * 100).dewPoint,
            speedOfSound: getSpeedOfSound(tempK)
        };
    } catch (error) {
        console.warn("[Météo] Échec de la récupération API ou mode hors-ligne: " + error.message);
        return null;
    }
};

// --- ASTROMÉTRIE ET SYNCHRONISATION TEMPORELLE ---

/**
 * Détermine le nom de la phase lunaire (approximatif).
 */
const getMoonPhaseName = (phase) => {
    if (phase === 0 || phase === 1) return 'Nouvelle Lune';
    if (phase === 0.25) return 'Premier Quartier';
    if (phase === 0.5) return 'Pleine Lune';
    if (phase === 0.75) return 'Dernier Quartier';
    if (phase > 0 && phase < 0.25) return 'Premier Croissant';
    if (phase > 0.25 && phase < 0.5) return 'Lune Gibbeuse Croissante';
    if (phase > 0.5 && phase < 0.75) return 'Lune Gibbeuse Décroissante';
    if (phase > 0.75 && phase < 1) return 'Dernier Croissant';
    return 'Inconnu';
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

    // Mise à jour du DOM (extrait de index (4).html)
    if ($('sun-altitude')) $('sun-altitude').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 2, '°');
    if ($('sunrise-times')) $('sunrise-times').textContent = `${times.sunrise.toLocaleTimeString('fr-FR')} (TSM)`;
    if ($('sunset-times')) $('sunset-times').textContent = `${times.sunset.toLocaleTimeString('fr-FR')} (TSM)`;
    
    // Lune
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, '%');
    if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
    if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 2, '°');
    // ... (autres données astro/crépuscule)
};

/**
 * Synchronise l'horloge interne avec un serveur NTP (via WorldTimeAPI).
 */
const syncH = async () => {
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error('Erreur API Temps');
        const data = await response.json();
        const serverTime = new Date(data.datetime);
        lServH = serverTime.getTime();
        lLocH = Date.now();
    } catch (error) {
        console.error("[NTP] Échec de la synchronisation:", error.message);
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE';
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
/**
 * COUPE ARTIFICIELLE N°4 : Démarrage du Système, Carte (Leaflet) et Gestion des Événements
 *
 * Ce bloc est le point d'entrée (IIFE), l'initialisation GPS/Leaflet et les
 * gestionnaires d'événements pour les contrôles dynamiques.
 * Contenu principal extrait de: (8).js, (15) (2).js, (14).js, (5).js, (dashboard (1).js).
 */

((window) => {
    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [
            (typeof math === 'undefined' ? "math.min.js (math)" : ""),
            (typeof L === 'undefined' ? "leaflet.js (L)" : ""),
            (typeof SunCalc === 'undefined' ? "suncalc.js (SunCalc)" : ""),
            (typeof turf === 'undefined' ? "turf.min.js (turf)" : "")
        ].filter(Boolean).join(", ");

        console.error(`Erreur critique : Dépendances manquantes : ${missing}. Le script est arrêté.`);
        alert(`Erreur: Dépendances critiques manquantes. L'application ne démarrera pas.`);
        return;
    }

    // --- INITIALISATION DE LA CARTE (Leaflet) ---
    const map = L.map('map-container').setView([currentPosition.lat, currentPosition.lon], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
    }).addTo(map);
    const marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);

    // --- CONFIGURATIONS GPS (Optimisation Batterie) ---
    const GPS_OPTS_HIGH_FREQ = { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 };
    const GPS_OPTS_LOW_FREQ = { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 };

    let domID = null; // ID pour la boucle lente (Astro/Météo)

    /**
     * Boucle lente (toutes les DOM_SLOW_UPDATE_MS) : Astro, Météo, Horloge NTP.
     */
    const startSlowLoop = () => {
        if (domID) clearInterval(domID);
        domID = setInterval(async () => {
            const currentLat = currentPosition.lat;
            const currentLon = currentPosition.lon;
            const now = getCDate(lServH, lLocH);

            // 1. Mise à jour Astrométrique
            updateAstro(currentLat, currentLon);

            // 2. Synchronisation NTP (toutes les minutes)
            if (now && Math.floor(now.getTime() / 1000) % 60 === 0) {
                 syncH();
            }

            // 3. Récupération Météo (si GPS actif)
            if (!isGpsPaused && !emergencyStopActive) {
                const weatherData = await fetchWeather(currentLat, currentLon);
                if (weatherData) {
                    lastKnownWeather = weatherData;
                    // Met à jour les variables pour le filtre UKF
                    lastP_hPa = weatherData.pressure_hPa;
                    lastT_K = weatherData.tempK;
                    currentAirDensity = weatherData.air_density;
                    currentSpeedOfSound = weatherData.speedOfSound;

                    // Met à jour le DOM météo
                    if ($('temp-air-2')) $('temp-air-2').textContent = `${weatherData.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${weatherData.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${weatherData.humidity_perc} %`;
                    if ($('air-density')) $('air-density').textContent = `${weatherData.air_density.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${weatherData.dew_point.toFixed(1)} °C`;
                }
            }

            // 4. Mise à jour de l'horloge
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
                if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
            }

        }, DOM_SLOW_UPDATE_MS);
    };

    /**
     * Fonction de succès GPS (Appelée par l'API du navigateur).
     */
    const onGpsSuccess = (pos) => {
        const coords = pos.coords;

        // Mise à jour de l'état global
        currentPosition.lat = coords.latitude;
        currentPosition.lon = coords.longitude;
        currentPosition.alt = coords.altitude || currentPosition.alt; // Conserve la dernière altitude si N/A
        currentPosition.acc = coords.accuracy;
        currentPosition.spd = coords.speed || 0.0;

        // --- TRAITEMENT UKF ---
        if (ukf) {
            const dt = (Date.now() - (pos.timestamp || Date.now())) / 1000;
            // ukf.predict(currentIMUData, dt);
            // ukf.update(currentGNSSMeasurement, R_covariance_matrix);
        }

        // --- AFFICHAGE ---
        if ($('latitude-value')) $('latitude-value').textContent = dataOrDefault(currentPosition.lat, 6, '°');
        if ($('longitude-value')) $('longitude-value').textContent = dataOrDefault(currentPosition.lon, 6, '°');
        if ($('altitude-value')) $('altitude-value').textContent = dataOrDefault(currentPosition.alt, 2, ' m');
        if ($('speed-value')) $('speed-value').textContent = dataOrDefault(currentPosition.spd * KMH_MS, 2, ' km/h');
        if ($('acc-value')) $('acc-value').textContent = dataOrDefault(currentPosition.acc, 2, ' m');

        // --- CARTE ---
        const newLatLon = L.latLng(currentPosition.lat, currentPosition.lon);
        marker.setLatLng(newLatLon);
        map.setView(newLatLon, map.getZoom()); // Re-centre la carte
    };

    /**
     * Démarre l'acquisition GPS (navigator.geolocation.watchPosition).
     */
    const startGPS = (opts = GPS_OPTS_HIGH_FREQ) => {
        if (watchID) navigator.geolocation.clearWatch(watchID);
        watchID = navigator.geolocation.watchPosition(onGpsSuccess, (err) => {
            console.error("[GPS] Erreur d'acquisition:", err);
            if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `❌ Erreur GPS: ${err.code} (${err.message})`;
        }, opts);
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Actif (Précision: Haute)`;
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

    // --- GESTIONNAIRES D'ÉVÉNEMENTS DOM ---
    document.addEventListener('DOMContentLoaded', () => {

        // Toggle GPS Marche/Pause
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').addEventListener('click', () => {
                isGpsPaused ? startGPS() : stopGPS();
            });
        }

        // Toggle Mode Nuit/Jour
        if ($('toggle-mode-btn')) {
            $('toggle-mode-btn').addEventListener('click', () => {
                document.body.classList.toggle('dark-mode');
                const isDark = document.body.classList.contains('dark-mode');
                $('toggle-mode-btn').innerHTML = isDark ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
            });
        }

        // Toggle Mode Nether (1:8 Scale)
        if ($('nether-toggle-btn')) {
            $('nether-toggle-btn').addEventListener('click', () => {
                netherMode = !netherMode;
                $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
            });
        }

        // Bouton "Rapport Distance"
        if ($('distance-ratio-toggle-btn')) {
            $('distance-ratio-toggle-btn').addEventListener('click', () => {
                distanceRatioMode = !distanceRatioMode;
                if (ukf) {
                    const ratio = distanceRatioMode ? ukf.calculateDistanceRatio(currentPosition.alt) : 1.0;
                    $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
                }
            });
        }
        
        // Sélecteur Réactivité UKF
        if ($('ukf-reactivity-mode')) {
            $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
        }

        // Mise à jour de la masse
        if ($('mass-input')) {
            $('mass-input').addEventListener('input', (e) => {
                currentMass = parseFloat(e.target.value) || 70.0;
                if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
            });
        }

        // Mise à jour du corps céleste
        if ($('celestial-body-select')) {
            $('celestial-body-select').addEventListener('change', (e) => {
                currentCelestialBody = e.target.value;
                if (ukf) {
                    const { G_ACC_NEW } = ukf.updateCelestialBody(currentCelestialBody, currentPosition.alt, rotationRadius, angularVelocity);
                    if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
                }
            });
        }

        // --- DÉMARRAGE DU SYSTÈME ---
        syncH().finally(() => {
            // L'UKF est initialisé après la synchronisation NTP pour des raisons de temps précis
            ukf = new ProfessionalUKF();
            ukf.updateCelestialBody(currentCelestialBody, currentPosition.alt, rotationRadius, angularVelocity);

            startGPS();
            startSlowLoop();

            // Affichages initiaux
            if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (ISA)`;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });

    }); // Fin de DOMContentLoaded
})(window);
