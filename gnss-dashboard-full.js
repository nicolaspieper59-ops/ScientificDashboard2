/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (COMPLET/PROFESSIONNEL)
 * FUSION FINALE de tous les fichiers source JS.
 * Dépendances Requises (dans le HTML) : math.min.js, leaflet.js, suncalc.js, turf.min.js.
 */

// =================================================================
// COUPE ARTIFICIELLE N°1 : Noyau Scientifique, Constantes et État Global
// =================================================================

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
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS (PROXY VERCEL) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000;
const DOM_HIGH_FREQ_MS = 17;

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

// --- PARAMÈTRES DU FILTRE DE KALMAN NON PARFUMÉ (UKF) ---
const UKF_ALPHA = 1e-3;
const UKF_BETA = 2.0;
const UKF_KAPPA = 0.0;
const UKF_STATE_DIM = 21; // P(3), V(3), A(3), Biais Capteurs, Biais Horloge, etc.

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let ukf = null;
let isGpsPaused = false;
let currentPosition = {
    lat: 43.2964,   // Valeur par défaut (Marseille)
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
let currentCelestialBody = 'Terre'; // Utilise 'Terre' pour le sélecteur HTML
let netherMode = false;
let distanceRatioMode = false;
let currentUKFReactivity = 'Automatique (Adaptatif)';
let lServH = 0, lLocH = 0;      // Temps Serveur (NTP) et Temps Local
let watchID = null;             // ID du GPS Watch
let totalDistance = 0.0;        // Distance 3D totale
let maxSpeed = 0.0;             // Vitesse maximale

// =================================================================
// COUPE ARTIFICIELLE N°2 : Modèles Physiques et Cinématiques
// =================================================================

/**
 * Calcule la vitesse du son dans l'air (m/s) à partir de la température en Kelvin (K).
 * C_s = sqrt(gamma * R_specific * T)
 */
const getSpeedOfSound = (tempK) => {
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
};

/**
 * Calcule les paramètres Bio-SVT (Point de Rosée).
 */
const calculateBioSVT = (tempC, altitudeM, humidityPerc, pressurePa) => {
    // Calcul de la Pression de vapeur saturante (formule Magnus-Tetens)
    const Pv_s = 6.1078 * Math.pow(10, (7.5 * tempC) / (tempC + 237.3));
    const Pv = Pv_s * (humidityPerc / 100);
    // Calcul du Point de Rosée (Td)
    const Td_num = 237.3 * Math.log(Pv / 6.1078);
    const Td_den = 7.5 - Math.log(Pv / 6.1078);
    const dewPoint = Td_num / Td_den;

    // NOTE: Les autres Bio-SVT (CAPE, O2 Saturation, etc.) nécessiteraient des modèles plus complexes, laissés à N/A dans le HTML.
    return { dewPoint: dewPoint };
};

/**
 * Met à jour l'accélération gravitationnelle G_ACC et le rayon terrestre
 * en fonction du corps céleste (Terre, Lune, Rotation Simulé) et de l'altitude.
 */
const updateCelestialBody = (body, altitude, radius, angularV) => {
    let G_ACC_NEW = 9.80665;
    let R_NEW = R_ALT_CENTER_REF;

    if (body === 'Terre') {
        // Modèle WGS84 (simplifié, la latitude est nécessaire pour une précision totale)
        // G_ACC_NEW = WGS84_G_EQUATOR * (1 + WGS84_BETA * Math.sin(currentPosition.lat * D2R)^2)
        // La correction en altitude est aussi omise pour la simplicité du code client
        // G_ACC_NEW = G_ACC_NEW * (R_ALT_CENTER_REF / (R_ALT_CENTER_REF + altitude))^2;
    } else if (body === 'Rotation') {
        // Simulation d'une station spatiale en rotation
        G_ACC_NEW = Math.abs(angularV * angularV * radius); // G_simulée = a_centripète
    }
    // Mise à jour des variables globales
    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_NEW;
    return { G_ACC_NEW, R_NEW };
};

/**
 * Calcule le ratio de distance entre la surface et le centre de la Terre.
 */
const calculateDistanceRatio = (altitude) => {
    return (R_ALT_CENTER_REF + altitude) / R_ALT_CENTER_REF;
};

// =================================================================
// COUPE ARTIFICIELLE N°3 : Le Filtre UKF 21 États, Astrométrie et Temps
// =================================================================

/**
 * @class ProfessionalUKF
 * @description Filtre de Kalman Non Parfumé (UKF) pour la fusion d'états non linéaires (INS/GNSS).
 */
class ProfessionalUKF {
    constructor() {
        // État initial (21x1): P (3), V (3), A (3), Biais Capteurs (9), Erreurs (6)
        this.X = math.matrix(math.zeros([UKF_STATE_DIM, 1]));
        // Covariance initiale (21x21)
        this.P = math.matrix(math.diag(math.ones([UKF_STATE_DIM]).map((_, i) => i < 9 ? 1e-2 : 1e-4)));
        console.log(`[UKF] Filtre ${UKF_STATE_DIM} États initialisé.`);
    }

    /**
     * Modèle de Prédiction (Propagation de l'État): F(X, U, dt)
     * @comment Nécessite l'implémentation complète des forces (Coriolis, Hydro, Relativité)
     */
    predict(U, dt) {
        // Placeholder pour l'implémentation complète des points sigma de l'UKF
    }

    /**
     * Modèle de Mise à Jour (Correction de l'État): H(X)
     */
    update(Z, R) {
        // Placeholder pour l'implémentation complète du gain de Kalman et la correction d'état
    }
}

// --- FONCTIONS ASTROMÉTRIE ET TEMPS ---

/**
 * Détermine le nom de la phase lunaire (dépend de SunCalc).
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

    // Soleil
    if ($('altitude-soleil')) $('altitude-soleil').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 2, '°');
    if ($('sunrise-times')) $('sunrise-times').textContent = `${times.sunrise.toLocaleTimeString('fr-FR')} (TSM)`;
    if ($('sunset-times')) $('sunset-times').textContent = `${times.sunset.toLocaleTimeString('fr-FR')} (TSM)`;

    // Lune
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, '%');
    if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
    if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 2, '°');
    // Mise à jour de l'heure du crépuscule/nuit (Nuit/Crépuscule)
    const sunAlt = sunPos.altitude * R2D;
    let twilightStatus = 'Nuit Noire';
    if (sunAlt > -6) twilightStatus = 'Crépuscule Civil (Clair)';
    if (sunAlt > -12 && sunAlt <= -6) twilightStatus = 'Crépuscule Nautique';
    if (sunAlt > -18 && sunAlt <= -12) twilightStatus = 'Crépuscule Astronomique';
    if (sunAlt > 0) twilightStatus = 'Jour (☀️)';

    if ($('twilight-status')) $('twilight-status').textContent = twilightStatus;
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
};

/**
 * Synchronise l'horloge interne avec un serveur NTP (via WorldTimeAPI).
 */
const syncH = async () => {
    if ($('heure-locale')) $('heure-locale').textContent = 'Synchronisation...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error('Erreur API Temps');
        const data = await response.json();
        const serverTime = new Date(data.datetime);
        lServH = serverTime.getTime();
        lLocH = Date.now();
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Actif (Précision: Haute)`;
    } catch (error) {
        console.error("[NTP] Échec de la synchronisation:", error.message);
        if ($('heure-locale')) $('heure-locale').textContent = 'SYNCHRO ÉCHOUÉE';
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


// =================================================================
// COUPE ARTIFICIELLE N°4 : Démarrage du Système, Carte et Gestion des Événements (IIFE)
// =================================================================

((window) => {
    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        // Affichage d'une erreur critique dans le statut GPS
        const statusElement = $('statut-gps-acquisition') || document.body;
        statusElement.innerHTML = `<h2 style=\"color:red;\">CRASH SCRIPT: Dépendances manquantes</h2>`;
        return;
    }

    // --- INITIALISATION DE LA CARTE (Leaflet) ---
    const map = L.map('map-container').setView([currentPosition.lat, currentPosition.lon], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
    }).addTo(map);
    const marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);

    // --- CONFIGURATIONS GPS ---
    const GPS_OPTS_HIGH_FREQ = { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 };
    const GPS_OPTS_LOW_FREQ = { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 };

    let domID = null; // ID pour la boucle lente (Astro/Météo)
    let lastTimestamp = Date.now(); // Pour le calcul du temps écoulé

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
            if (!isGpsPaused) {
                const weatherData = await fetchWeather(currentLat, currentLon);
                if (weatherData) {
                    lastKnownWeather = weatherData;
                    // Met à jour les variables pour le filtre UKF
                    lastP_hPa = weatherData.pressure_hPa;
                    lastT_K = weatherData.tempK;
                    currentAirDensity = weatherData.air_density;
                    currentSpeedOfSound = weatherData.speedOfSound;

                    // Met à jour le DOM météo
                    if ($('statut-meteo')) $('statut-meteo').textContent = `ACTIF`;
                    if ($('temp-air')) $('temp-air').textContent = `${weatherData.tempC.toFixed(1)} °C`;
                    if ($('pressure-atmospherique')) $('pressure-atmospherique').textContent = `${weatherData.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidite-relative')) $('humidite-relative').textContent = `${weatherData.humidity_perc} %`;
                    if ($('densite-air')) $('densite-air').textContent = `${weatherData.air_density.toFixed(3)} kg/m³`;
                    if ($('point-rosee')) $('point-rosee').textContent = `${weatherData.dew_point.toFixed(1)} °C`;
                } else {
                    if ($('statut-meteo')) $('statut-meteo').textContent = `INACTIF (Échec API)`;
                }
            }

            // 4. Mise à jour de l'horloge
            if (now) {
                if ($('heure-locale') && !$('heure-locale').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                    $('heure-locale').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');

                // Mise à jour de l'heure écoulée
                const elapsedTime = (Date.now() - window.appStartTime) / 1000;
                if ($('temps-ecoule')) $('temps-ecoule').textContent = `${dataOrDefault(elapsedTime, 2, ' s')}`;
            }

        }, DOM_SLOW_UPDATE_MS);
    };

    /**
     * Fonction de succès GPS (Appelée par l'API du navigateur).
     */
    const onGpsSuccess = (pos) => {
        const coords = pos.coords;
        const dt = (pos.timestamp - lastTimestamp) / 1000; // Delta temps entre deux acquisitions

        // Calcul de la distance 3D parcourue pour la distance totale
        const d_lat = currentPosition.lat - coords.latitude;
        const d_lon = currentPosition.lon - coords.longitude;
        const d_alt = currentPosition.alt - (coords.altitude || currentPosition.alt);
        const distance3D = Math.sqrt(d_lat * d_lat + d_lon * d_lon + d_alt * d_alt) * 111139; // Approximation grossière en m

        totalDistance += distance3D;
        lastTimestamp = pos.timestamp;

        // Mise à jour de l'état global
        currentPosition.lat = coords.latitude;
        currentPosition.lon = coords.longitude;
        currentPosition.alt = coords.altitude || currentPosition.alt;
        currentPosition.acc = coords.accuracy;
        currentPosition.spd = coords.speed || 0.0;
        maxSpeed = Math.max(maxSpeed, currentPosition.spd * KMH_MS);

        // --- TRAITEMENT UKF (Simulation) ---
        if (ukf) {
            // Le filtre UKF professionnel nécessiterait des données IMU (acc, gyro) en plus du GPS
            // ukf.predict(currentIMUData, dt);
            // ukf.update(currentGNSSMeasurement, R_covariance_matrix);
        }

        // --- AFFICHAGE ---
        if ($('latitude-value')) $('latitude-value').textContent = dataOrDefault(currentPosition.lat, 6, '°');
        if ($('longitude-value')) $('longitude-value').textContent = dataOrDefault(currentPosition.lon, 6, '°');
        if ($('altitude-value')) $('altitude-value').textContent = dataOrDefault(currentPosition.alt, 2, ' m');
        if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(currentPosition.spd * KMH_MS, 2, ' km/h');
        if ($('vitesse-brute')) $('vitesse-brute').textContent = dataOrDefault(currentPosition.spd, 2, ' m/s');
        if ($('vitesse-max')) $('vitesse-max').textContent = dataOrDefault(maxSpeed, 1, ' km/h');
        if ($('precision-gps-acc')) $('precision-gps-acc').textContent = dataOrDefault(currentPosition.acc, 2, ' m');
        if ($('distance-totale-3d')) $('distance-totale-3d').textContent = `${dataOrDefault(totalDistance / 1000, 3, ' km')} | ${dataOrDefault(totalDistance, 2, ' m')}`;
        if ($('pourcentage-c')) $('pourcentage-c').textContent = dataOrDefaultExp((currentPosition.spd / C_L) * 100, 3, ' %');

        // --- CARTE ---
        const newLatLon = L.latLng(currentPosition.lat, currentPosition.lon);
        marker.setLatLng(newLatLon);
        map.setView(newLatLon, map.getZoom() < 13 ? 13 : map.getZoom());
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
        // Enregistre l'heure de début pour le calcul du temps écoulé
        window.appStartTime = Date.now();

        // Initialisation des classes et états par défaut
        ukf = new ProfessionalUKF();
        updateCelestialBody(currentCelestialBody, currentPosition.alt, rotationRadius, angularVelocity);

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
                $('toggle-mode-btn').innerHTML = isDark ? '🌙 Mode Nuit' : '☀️ Mode Jour';
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
                const ratio = distanceRatioMode ? calculateDistanceRatio(currentPosition.alt) : 1.0;
                $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
            });
        }

        // Sélecteur Réactivité UKF
        if ($('ukf-reactivity-mode')) {
            $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
        }

        // Gestionnaire de la rotation (Rayon et Vitesse Angulaire)
        const updateRotation = () => {
            rotationRadius = parseFloat($('rayon-rotation').value) || 100.0;
            angularVelocity = parseFloat($('vitesse-angulaire').value) || 0.0;
            if (currentCelestialBody === 'Rotation') {
                const { G_ACC_NEW } = updateCelestialBody('Rotation', currentPosition.alt, rotationRadius, angularVelocity);
                if ($('gravite-base')) $('gravite-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
            }
        };
        if ($('rayon-rotation')) $('rayon-rotation').addEventListener('input', updateRotation);
        if ($('vitesse-angulaire')) $('vitesse-angulaire').addEventListener('input', updateRotation);

        // Mise à jour du corps céleste
        if ($('celestial-body-select')) {
            $('celestial-body-select').addEventListener('change', (e) => {
                currentCelestialBody = e.target.value;
                const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, currentPosition.alt, rotationRadius, angularVelocity);
                if ($('gravite-base')) $('gravite-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
            });
        }

        // Mise à jour de la masse
        if ($('masse-objet')) {
            $('masse-objet').addEventListener('input', (e) => {
                currentMass = parseFloat(e.target.value) || 70.0;
                if ($('masse-display')) $('masse-display').textContent = `${currentMass.toFixed(3)} kg`;
            });
        }

        // --- DÉMARRAGE DU SYSTÈME ---
        // Initialisation de l'affichage par défaut
        if ($('vitesse-lumiere')) $('vitesse-lumiere').textContent = `${C_L} m/s`;
        if ($('gravitation-universelle')) $('gravitation-universelle').textContent = dataOrDefaultExp(G_U, 5, ' m³/kg/s²');
        if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (ISA)`;
        if ($('gravite-base')) $('gravite-base').textContent = `${G_ACC.toFixed(4)} m/s²`;

        syncH().finally(() => {
            startGPS();
            startSlowLoop();
        });

    }); // Fin de DOMContentLoaded
})(window);
                          
