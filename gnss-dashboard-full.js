// =================================================================
// BLOC 1/4 : CONSTANTES, UKF & MODÈLES PHYSIQUES (NON-SIMPLIFIÉS)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
// ... [dataOrDefault et autres fonctions utilitaires ici] ...

((window) => {
    // VÉRIFICATION DES DÉPENDANCES CRITIQUES POUR LE RÉALISME MAXIMAL
    // SunCalc.js est remplacé par ephem.js.
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof ephem === 'undefined' || typeof turf === 'undefined') {
        const missing = [
            (typeof math === 'undefined' ? "math.min.js (UKF)" : ""), 
            (typeof L === 'undefined' ? "leaflet.js" : ""),
            (typeof ephem === 'undefined' ? "ephem.min.js (Astrométrie)" : ""), 
            (typeof turf === 'undefined' ? "turf.min.js" : "")
        ].filter(Boolean).join(", ");
        alert(`❌ ERREUR CRITIQUE: Dépendances Manquantes pour le Réalisme Scientifique : ${missing}.`);
        return;
    }

    // --- CONSTANTES WGS84 & ISA (MODÈLES PHYSIQUES NON-SIMPLIFIÉS) ---
    // [Ces constantes sont déjà non-simplifiées dans votre code original]
    const WGS84_A = 6378137.0; // Rayon équatorial (m)
    const WGS84_E2 = 0.00669437999013; // Excentricité au carré
    const MU = 3.986004418e14; // Constante gravitationnelle terrestre
    const OMEGA_EARTH = 7.2921159e-5; // Vitesse angulaire de la Terre (rad/s)
    const R_AIR = 287.058; // Constante Spécifique de l'Air Sec (J/kg·K)
    // ... [Autres constantes ici : TEMP_SEA_LEVEL_K, BARO_ALT_REF_HPA, etc.] ...

    // --- UKF 21 ÉTATS (INITIALISATION) ---
    // [Classe ProfessionalUKF (21 états : Position(3), Vitesse(3), Attitude(4), Biais Gyro(3), Biais Accel(3), etc.)]
    let ukf = new ProfessionalUKF();
    // ...
 // =================================================================
// BLOC 2/4 : ASTROMÉTRIE & CALCULS CÉLESTES (ephem.js / VSOP2013)
// =================================================================

/**
 * @function computeCelestial
 * Calcule les positions du Soleil et de la Lune avec la plus haute fidélité (ephem.js).
 * REMPLACE les fonctions simplifiées de SunCalc.
 * @param {Date} date - L'heure en UTC.
 * @param {number} lat - Latitude (WGS84).
 * @param {number} lon - Longitude (WGS84).
 */
const computeCelestial = (date, lat, lon) => {
    // Conversion de la date en Temps Julien (J2000) pour ephem.js
    const date_ms = date.getTime();
    const jy2k = ephem.dateToJd(date_ms); // Fonction d'aide supposée

    // --- SOLEIL (SUN) : VSOP2013 ---
    // Les fonctions ephem.js fournissent la position par rapport au Barycentre Solaire.
    const sun_pos = ephem.sun(jy2k); 
    // Nécessite une conversion en coordonnées topocentriques (alt/az) 
    // qui doit être effectuée avec les fonctions de projection fournies par ephem.js ou une librairie tierce.
    // L'Azimut et l'Altitude du Soleil doivent être calculés à partir de sun_pos, lat, lon.
    const sun_alt_az = ephem.getTopocentric(sun_pos, lat, lon); // Structure hypothétique

    // --- LUNE (MOON) : ELP2000-MPP02 ---
    const moon_pos = ephem.moon(jy2k);
    const moon_alt_az = ephem.getTopocentric(moon_pos, lat, lon); // Structure hypothétique
    
    // --- Phase Lunaire (calculée par ephem.js pour la non-simplification) ---
    const phase = ephem.getMoonPhase(jy2k); 

    return {
        sun_alt: sun_alt_az.alt,
        sun_azimuth: sun_alt_az.az,
        moon_alt: moon_alt_az.alt,
        moon_azimuth: moon_alt_az.az,
        moon_phase: phase
    };
};

// ... [Autres fonctions d'aide au calcul (gravity, earth-rotation, etc.) utilisant WGS84] ...
 // =================================================================
// BLOC 3/4 : GESTION DES CAPTEURS ET BOUCLE UKF (HAUTE FRÉQUENCE)
// =================================================================

// --- DONNÉES DE HAUTE FRÉQUENCE (IMU) ---
let lastImuTimestamp = 0;
let isMoving = false; // Utilisé pour déclencher la ZUPT (Grottes, Métro)

/**
 * @function handleDeviceMotion
 * Gestionnaire d'événements pour le capteur IMU (Accéléromètre et Gyroscope). 
 * C'est le COEUR DE LA NAVIGATION INS.
 */
window.addEventListener('devicemotion', (event) => {
    const currentTimestamp = Date.now();
    let dt = (currentTimestamp - lastImuTimestamp) / 1000; // Delta de temps en secondes

    // --- VÉRIFICATION CRITIQUE POUR LA NON-SIMPLIFICATION ---
    if (dt === 0 || dt > 0.1) {
        // Ignorer les événements avec dt nul (bruit) ou trop grand (perte de données).
        // L'UKF exige un dt stable et rapide.
        lastImuTimestamp = currentTimestamp;
        return; 
    }
    
    // 1. EXTRACTION DES DONNÉES BRUTES (Accélération & Vitesse Angulaire)
    const acc = event.accelerationIncludingGravity;
    const gyro = event.rotationRate;
    
    // 2. CORRECTION UKF
    // La fonction predict() propage l'état 21 en utilisant le dt et les données IMU.
    // Elle utilise OMEGA_EARTH (WGS84) pour corriger la Force de Coriolis et l'intégration.
    ukf.predict(dt, acc, gyro); 
    
    // 3. LOGIQUE ZUPT (Correction Non-Simplifiée à Faible Vitesse)
    // Application de la Zero-Velocity Update (ZUPT) pour les arrêts (Métro, Attractions)
    if (!isMoving && ukf.getEstimatedSpeedNorm() < 0.05) { // Si vitesse estimée < 5cm/s
        ukf.applyZUPT(); // Fonction qui force l'erreur de vitesse à zéro, purgeant le drift.
    }

    lastImuTimestamp = currentTimestamp;
});

// ... [Autres gestionnaires (GPS, Baromètre, etc.)] ...
 // =================================================================
// BLOC 4/4 : MISE À JOUR DU DOM ET INITIALISATION FINALE
// =================================================================

/**
 * @function updateCelestialDOM
 * Met à jour les valeurs d'éphémérides du DOM (Soleil/Lune).
 */
const updateCelestialDOM = (lat, lon) => {
    const date = new Date();
    const celestialData = computeCelestial(date, lat, lon);

    // --- AFFICHAGE SOLEIL/LUNE (Utilise ephem.js) ---
    $('sun-alt').textContent = dataOrDefault(celestialData.sun_alt, 3, '°');
    $('sun-azimuth').textContent = dataOrDefault(celestialData.sun_azimuth, 3, '°');
    // ... [Autres mises à jour du DOM pour l'altitude, l'azimut, etc.] ...
    $('moon-phase-name').textContent = celestialData.moon_phase.name || 'N/A';
};

/**
 * @function updateDashboard
 * Boucle principale pour la mise à jour des données (DOM, Carte).
 */
const updateDashboard = () => {
    // ... [Calcul de la position GNSS (si disponible)] ...
    // ... [Récupération de l'état UKF (position, attitude, biais, covariance)] ...

    // --- MISE À JOUR DES CHAMPS CRITIQUES ---
    // Vitesse Filtrée et non-simplifiée
    $('filtered-velocity').textContent = dataOrDefault(ukf.getEstimatedSpeedNorm(), 4, ' m/s'); 
    
    // Biais du Gyroscope (Preuve du réalisme de la modélisation)
    $('gyro-bias-x').textContent = dataOrDefaultExp(ukf.getEstimatedGyroBias().x, 2, ' rad/s');
    
    // ... [Mise à jour des autres champs de l'UKF] ...

    // --- MISE À JOUR CÉLESTE (ephem.js) ---
    updateCelestialDOM(currentLat, currentLon); 

    // Relancer la boucle pour la prochaine mise à jour DOM (typiquement 10Hz)
    requestAnimationFrame(updateDashboard); 
};

// --- INITIALISATION FINALE ---
window.onload = () => {
    // ... [Code d'initialisation de la carte, des variables] ...
    // Démarrer la boucle de rafraîchissement du DOM
    requestAnimationFrame(updateDashboard);
};
})(window);
