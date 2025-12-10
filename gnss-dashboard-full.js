// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// INTÉGRATION FINALE V6 - Mise à jour complète de tous les IDs DOM
// DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML) :
// - math.min.js
// - lib/ukf-lib.js (DOIT contenir la classe ProfessionalUKF)
// - lib/astro.js (DOIT contenir les fonctions getJulianDay, formatHours, getAstroData)
// =================================================================

((window) => {
    "use strict";

    // --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
    const KMH_MS = 3.6;
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C
    const BARO_ALT_REF_HPA = 1013.25;
    const RHO_SEA_LEVEL = 1.225; // kg/m³
    const G_U = 6.67430e-11; // Constante gravitationnelle universelle

    // Fonction d'aide pour l'accès rapide aux IDs
    const $ = id => document.getElementById(id);

    // Fonctions d'aide pour l'affichage (avec gestion des valeurs nulles/indéfinies)
    const dataOrDefault = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val)) {
            return (decimals === 0 ? '0' : '0.00') + suffix;
        }
        return val.toFixed(decimals) + suffix;
    };
    
    // Pour les petits nombres (format scientifique)
    const dataOrDefaultExp = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val)) {
            const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
            return zeroDecimals + 'e+0' + suffix;
        }
        return val.toExponential(decimals) + suffix;
    };


    // --- BLOC 2 : ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---

    let ukf; // L'instance du filtre UKF (ProfessionalUKF)
    let ukfState = {
        lat: 43.2964, lon: 5.3697, alt: 0.0, speed: 0.0, acc: 10.0,
        pos_uncertainty: 5.0, vel_uncertainty: 0.5, clock_drift: 1.0e-8,
        speed_correction: 0.0, alt_correction: 0.0, ukf_error: 0.1
    };
    let currentPosition = { lat: 43.2964, lon: 5.3697, total_distance: 0.0, course: 0.0, spd: 0.0 };
    let maxSpeed = 0.0;
    let kAlt = 0.0; // Altitude filtrée/estimée (utilisée pour les calculs physiques)
    let lastIMUData = { accelX: 0.0, accelY: 0.0, accelZ: 9.81, rotAlpha: 0.0, rotBeta: 0.0, rotGamma: 0.0 };
    let currentUKFReactivity = "NORMAL";
    let currentMass = 70.0;
    let G_ACC = 9.80665; // Gravité initiale (sera mise à jour par WGS84)
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 343.0; // Vitesse du son initiale (m/s)
    let tempAirC = 15.0; // Température initiale pour affichage
    let pressure_hPa = BARO_ALT_REF_HPA; // Pression initiale
    let humidity_perc = 50; // Humidité initiale
    let dewPointC = 4.8; // Point de rosée initial
    let lServH = new Date().getTime(); // Horloge Serveur (NTP)
    let lLocH = new Date().getTime(); // Horloge Locale
    let astroData = {}; // Données Astro (Soleil/Lune)
    let currentCelestialBody = 'EARTH';
    let rotationRadius = 100.0;
    let angularVelocity = 0.0;


    // --- BLOC 3 : STUBS ET FONCTIONS SCIENTIFIQUES EXTERNES (NÉCESSAIRES) ---

    // La fonction doit être définie dans ukf-lib.js (ou être une simulation)
    const updateUKF = (newGpsData, imuData) => { /* Logic d'estimation UKF */ }; 

    // La fonction doit être définie dans astro.js (ou être une simulation)
    const getAstroData = (lat, lon, date) => { /* Logic de calcul solaire/lunaire */ };

    // Placeholder pour la synchro NTP
    const getCDate = (lServH, lLocH) => { 
        return new Date(lServH + (new Date().getTime() - lLocH));
    };

    // Placeholder pour le temps GPS (semaine/secondes)
    const getGPSWeekTime = (date) => { 
        const GPS_EPOCH = new Date(Date.UTC(1980, 0, 6, 0, 0, 0));
        const msSinceEpoch = date.getTime() - GPS_EPOCH.getTime();
        const secondsSinceEpoch = msSinceEpoch / 1000;
        const secondsPerWeek = 604800;
        const week = Math.floor(secondsSinceEpoch / secondsPerWeek);
        const time = secondsSinceEpoch % secondsPerWeek;
        return { week, time };
    };

    // Placeholder pour le calcul de la vitesse du son
    const getSpeedOfSound = (tempK) => Math.sqrt(1.4 * 287.058 * tempK); // 1.4 = Gamma Air, 287.058 = R_air
    
    // Placeholder pour la gestion de la gravité
    const updateCelestialBody = (body, alt, radius, velocity) => {
        if (body === 'ROTATING') {
            const rot_acc = radius * velocity * velocity;
            const G_ACC_NEW = 9.80665 - rot_acc;
            G_ACC = G_ACC_NEW;
            return { G_ACC_NEW };
        }
        // Logique plus complexe pour EARTH/MARS/MOON ici
        G_ACC = 9.80665;
        return { G_ACC_NEW: 9.80665 }; 
    };

    // Stubs d'initialisation
    const syncH = () => { /* Logique de synchronisation NTP */ };
    const initGPS = () => { /* Logique de démarrage GPS */ };
    const setupEventListeners = () => { /* Logique pour les boutons/inputs */ };


    // --- BLOC 4 : FONCTION DE MISE À JOUR DU DOM (Cœur de la demande) ---

    /**
     * Met à jour tous les éléments du DOM avec les données du système (UKF, GPS, Astro, Météo, IMU).
     */
    function updateDashboardDOM() {
        
        // Vérification des dépendances (pour éviter les erreurs silencieuses)
        if (typeof $ === 'undefined' || typeof dataOrDefault === 'undefined' || typeof dataOrDefaultExp === 'undefined') {
            console.error("🔴 Erreur: Les fonctions d'aide ($, dataOrDefault, dataOrDefaultExp) sont manquantes.");
            return;
        }

        const lat = ukfState.lat;
        const lon = ukfState.lon;
        const alt = ukfState.alt;
        const speed = ukfState.speed;
        const acc = ukfState.acc;
        const imu = lastIMUData || {};

        // =========================================================
        // 1. GPS & UKF (Position, Vitesse, Précision)
        // =========================================================
        if ($('lat-display')) $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
        if ($('lon-display')) $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
        if ($('alt-display')) $('alt-display').textContent = dataOrDefault(alt, 1, ' m');
        if ($('speed-display')) $('speed-display').textContent = dataOrDefault(speed * KMH_MS, 2, ' km/h'); // m/s -> km/h
        
        // UKF Uncertainties & Clock
        if ($('accuracy-display')) $('accuracy-display').textContent = dataOrDefault(acc, 2, ' m');
        if ($('ukf-pos-uncertainty')) $('ukf-pos-uncertainty').textContent = dataOrDefaultExp(ukfState.pos_uncertainty, 2, ' m');
        if ($('ukf-vel-uncertainty')) $('ukf-vel-uncertainty').textContent = dataOrDefaultExp(ukfState.vel_uncertainty, 2, ' m/s');
        if ($('clock-drift')) $('clock-drift').textContent = dataOrDefaultExp(ukfState.clock_drift * 1e9, 2, ' ns/s');


        // =========================================================
        // 2. Horloge & Temps
        // =========================================================
        const now = getCDate(lServH, lLocH); 
        if (now) {
            if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            
            if (typeof getGPSWeekTime !== 'undefined') {
                const { week, time } = getGPSWeekTime(now); 
                if ($('gps-time')) $('gps-time').textContent = `W${week} T${dataOrDefault(time, 2, ' s')}`;
            }

            if (typeof getJulianDay !== 'undefined') {
                if ($('julian-day')) $('julian-day').textContent = dataOrDefault(getJulianDay(now), 4);
            }
        }


        // =========================================================
        // 3. IMU (Inertial)
        // =========================================================
        // Note: Les IDs doivent être mis à jour par la boucle de capteur, ici on affiche juste l'état.
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(imu.accelX, 2, ' m/s²');
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(imu.accelY, 2, ' m/s²');
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(imu.accelZ, 2, ' m/s²');
        if ($('rot-alpha')) $('rot-alpha').textContent = dataOrDefault(imu.rotAlpha * R2D, 2, ' °/s'); // Conversion en deg/s pour l'affichage
        if ($('rot-beta')) $('rot-beta').textContent = dataOrDefault(imu.rotBeta * R2D, 2, ' °/s');
        if ($('rot-gamma')) $('rot-gamma').textContent = dataOrDefault(imu.rotGamma * R2D, 2, ' °/s');


        // =========================================================
        // 4. Carte & UKF Corrections
        // =========================================================
        if ($('total-distance')) $('total-distance').textContent = dataOrDefault(currentPosition.total_distance / 1000, 3, ' km');
        if ($('course-display')) $('course-display').textContent = dataOrDefault(currentPosition.course, 1, ' °');
        if ($('max-speed')) $('max-speed').textContent = dataOrDefault(maxSpeed * KMH_MS, 2, ' km/h');

        if ($('speed-correction')) $('speed-correction').textContent = dataOrDefaultExp(ukfState.speed_correction, 2, ' m/s');
        if ($('alt-correction')) $('alt-correction').textContent = dataOrDefaultExp(ukfState.alt_correction, 2, ' m');
        if ($('ukf-error')) $('ukf-error').textContent = dataOrDefaultExp(ukfState.ukf_error, 3);
        if ($('ukf-reactivity')) $('ukf-reactivity').textContent = currentUKFReactivity;
        
        
        // =========================================================
        // 5. Météo & Physique
        // =========================================================
        if ($('temp-air-2')) $('temp-air-2').textContent = dataOrDefault(tempAirC, 1, ' °C');
        if ($('pressure-2')) $('pressure-2').textContent = dataOrDefault(pressure_hPa, 0, ' hPa');
        if ($('humidity-2')) $('humidity-2').textContent = dataOrDefault(humidity_perc, 0, ' %');
        if ($('dew-point')) $('dew-point').textContent = dataOrDefault(dewPointC, 1, ' °C');
        if ($('air-density')) $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
        
        // Gravité & Référentiels
        if ($('mass-display')) $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
        if ($('gravity-base')) $('gravity-base').textContent = dataOrDefault(G_ACC, 4, ' m/s²');
        // 'env-factor' est géré par l'événement 'change' sur un select, mais on s'assure qu'il y ait une valeur par défaut
        if ($('env-factor') && !$('env-factor').textContent) $('env-factor').textContent = 'Terre (x1.0)';


        // =========================================================
        // 6. Astrodynamique
        // =========================================================
        const astro = astroData || {};
        const sun = astro.sun || {};
        const moon = astro.moon || {};

        // Temps Solaire et sidéral
        if ($('tst-display')) $('tst-display').textContent = (typeof formatHours !== 'undefined') ? formatHours(astro.TST_HRS) : dataOrDefault(astro.TST_HRS, 4, ' H');
        if ($('mst-display')) $('mst-display').textContent = (typeof formatHours !== 'undefined') ? formatHours(astro.MST_HRS) : dataOrDefault(astro.MST_HRS, 4, ' H');
        if ($('eot-display')) $('eot-display').textContent = dataOrDefault(astro.EOT_MIN, 2, ' min');

        // Soleil
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sun.altitude * R2D, 2, ' °');
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sun.azimuth * R2D, 2, ' °');
        if ($('sun-declination')) $('sun-declination').textContent = dataOrDefault(sun.declination * R2D, 2, ' °');
        if ($('day-duration')) $('day-duration').textContent = dataOrDefault(sun.dayDuration, 2, ' H');
        if ($('sunrise-times')) $('sunrise-times').textContent = sun.sunrise || 'N/A';
        if ($('sunset-times')) $('sunset-times').textContent = sun.sunset || 'N/A';

        // Lune
        if ($('moon-phase-name')) $('moon-phase-name').textContent = moon.phaseName || 'N/A';
        if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moon.illumination * 100, 1, ' %');
        if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moon.altitude * R2D, 2, ' °');
        if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moon.azimuth * R2D, 2, ' °');
        if ($('moon-times')) $('moon-times').textContent = moon.times || 'N/A';
        if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(moon.distance / 1000, 0, ' km');

        // Mise à jour de la carte (implémentée via L.js si la librairie est chargée)
        if (typeof updateMap !== 'undefined') {
            updateMap(lat, lon, speed, currentPosition.course);
        }
    }

    // --- BLOC 5 : INITIALISATION DU SYSTÈME ---

    window.addEventListener('load', () => {
        
        // 0. Vérification des dépendances (peut être plus complet dans votre version)
        if (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') {
            ukf = new ProfessionalUKF();
        } else {
            console.warn("⚠️ UKF est désactivé. Vérifiez le chargement de math.min.js et ukf-lib.js.");
        }

        // 1. Initialisation des systèmes critiques
        syncH(); // Démarrer la synchro NTP
        initGPS(); // Démarrer le GPS
        setupEventListeners(); // Attacher les contrôles (si implémentés)
        updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity); // Init gravité
        
        // 2. Premier rafraîchissement des valeurs de Fallback
        updateDashboardDOM();

        // 3. Boucle principale de rafraîchissement (Haute Fréquence pour le DOM)
        setInterval(updateDashboardDOM, 250); // 4 fois par seconde

        // 4. Boucle lente pour les données externes (Météo/Astro)
        setInterval(() => {
            // Mise à jour Astro (utilise les coordonnées de ukfState/currentPosition)
            if (typeof getAstroData !== 'undefined') {
                astroData = getAstroData(ukfState.lat * D2R, ukfState.lon * D2R, getCDate(lServH, lLocH));
            }
            // Ici, vous lanceriez la fonction de mise à jour météo asynchrone (fetchWeather)
            // fetchWeather(ukfState.lat, ukfState.lon).then(data => { /* ... met à jour les variables météo ... */ });
        }, 5000); // Toutes les 5 secondes
    });

})(window);
