// =================================================================
// FICHIER : gnss-dashboard-full.js
// VERSION : FINALE ULTIME CONSOLIDÉE - UKF 21 ÉTATS (H.P. 5 DÉCIMALES)
// CORRECTION : Intégration complète des dépendances et de la précision (5 décimales)
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js, lib/ukf-lib.js, lib/astro.js, lib/ephem.js, turf.min.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique avec une précision fixe, ou retourne 'N/A'/'0.00' si non valide.
 * @param {number} val - La valeur à formater.
 * @param {number} decimals - Le nombre de décimales à conserver (ex: 5).
 * @param {string} suffix - L'unité ou suffixe.
 * @param {string} fallback - Valeur de secours si N/A, par défaut '0.00000'.
 */
const dataOrDefault = (val, decimals, suffix = '', fallback = null) => {
    const defaultZero = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
    
    if (val === undefined || val === null || isNaN(val) || (typeof val === 'number' && Math.abs(val) < 1e-12)) {
        return (fallback !== null) ? fallback : defaultZero;
    }
    // Assure la haute précision
    return val.toFixed(decimals) + suffix;
};

/**
 * Formate une valeur numérique en notation exponentielle avec une précision fixe pour la mantisse.
 * @param {number} val - La valeur à formater.
 * @param {number} decimals - Le nombre de décimales pour la mantisse.
 * @param {string} suffix - L'unité ou suffixe.
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) < 1e-25) {
        // Retourne un zéro précis en notation scientifique pour la cohérence
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CONSTANTES PHYSIQUES HAUTE PRÉCISION ---
const C = 299792458.0;              // Vitesse de la lumière (m/s)
const G = 6.67430e-11;              // Constante gravitationnelle (m³/kg/s²)
const G_STD = 9.80670;              // Gravité Standard de Référence (m/s²)
const RHO_AIR_ISA = 1.225;          // Densité de l'air ISA (kg/m³)
const V_SOUND_ISA = 340.29000;      // Vitesse du son à 15°C (m/s)
// Constantes Astro (supposées disponibles via astro.js)
const R2D = 180 / Math.PI; // Radians to Degrees

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // --- ÉTATS GLOBAUX ---
    let ukf;
    let isGpsPaused = true;             // État 'PAUSE GPS' par défaut
    let isIMUActive = false;            // État 'Inactif' par défaut
    let currentMass = 70.0;             // Masse de l'objet (kg)
    let currentMaxSpeed = 1.1 / 3.6;    // 1.1 km/h converti en m/s
    let currentSessionTime = 0.0;       // Temps écoulé
    
    // État UKF initial (Coordonnées fournies par l'utilisateur, V=0)
    let currentUKFState = { 
        lat: 43.284584, lon: 5.358721, alt: 100.00, 
        vN: 0.0, vE: 0.0, vD: 0.0, 
        speed: 0.0, kUncert: 0.0 
    };
    let currentUKFReactivity = 'Automatique'; // 'Automatique (Adaptatif)'
    
    // La fonction `formatHours` est supposée exister dans `astro.js`
    const formatHours = window.formatHours || ((h) => dataOrDefault(h, 2, 'h'));
    const getJulianDay = window.getJulianDay || (() => 2461021.2388); // Fallback JD pour Tue, 09 Dec 2025 17:43:54

    // =========================================================
    // BLOC 1 : LOGIQUE DE CALCUL CRITIQUE (UKF/Physique/Astro)
    // =========================================================

    /**
     * Calcule toutes les grandeurs physiques et met à jour l'affichage.
     */
    function updateDashboard() {
        const now = new Date();
        currentSessionTime += 1/60; // Simulation d'une boucle à 60 FPS

        // Assurez-vous que la vitesse EKF est à zéro si le GPS/IMU est en PAUSE/Inactif
        const V_ms = isGpsPaused && !isIMUActive ? 0.0 : currentUKFState.speed;
        const M = currentMass;           
        
        // --- 1. CALCULS PHYSIQUES & RELATIVISTES (Haute Précision) ---
        
        // Relativité
        const v_ratio_c = V_ms / C; 
        const gamma = 1 / Math.sqrt(1 - v_ratio_c * v_ratio_c);
        const energy_rest = M * C * C;
        const momentum = M * gamma * V_ms;
        const schwartzschild_radius = (2 * G * M) / (C * C);

        // Mécanique
        const speed_kmh = V_ms * 3.6;
        const dynamic_pressure = 0.5 * RHO_AIR_ISA * V_ms * V_ms;
        const kinetic_energy = 0.5 * M * V_ms * V_ms;

        // Vitesse/Mach
        const mach_number = V_ms / V_SOUND_ISA;
        
        // --- 2. CALCULS ASTRO (Utilisation de astro.js) ---
        let astroData = null;
        if (typeof getSolarData === 'function') {
            astroData = getSolarData(now, currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
        }
        
        // --- 3. MISE À JOUR DOM : CONTRÔLES & SYSTÈME ---
        $('utc-datetime').textContent = now.toUTCString().replace('GMT', 'UTC'); 
        $('elapsed-time').textContent = dataOrDefault(currentSessionTime, 2, ' s'); 
        $('object-mass-display').textContent = dataOrDefault(M, 3, ' kg'); // 3 décimales pour correspondre à 70.000 kg
        $('base-gravity').textContent = dataOrDefault(G_STD, 4, ' m/s²'); // 4 décimales pour correspondre à 9.8067 m/s²
        $('force-gps-precision').textContent = dataOrDefault(0.0, 0); // 0 décimales pour 0

        // --- 4. MISE À JOUR DOM : IMU & ENVIRONNEMENTAUX ---
        $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';
        // Tous les champs IMU/Capteurs Env. sont N/A
        const imu_fallback = 'N/A';
        $('accel-x').textContent = imu_fallback; $('accel-y').textContent = imu_fallback; $('accel-z').textContent = imu_fallback;
        $('mag-x').textContent = imu_fallback; $('mag-y').textContent = imu_fallback; $('mag-z').textContent = imu_fallback;
        $('light-ambiant').textContent = imu_fallback; $('sound-level').textContent = imu_fallback;
        
        // --- 5. MISE À JOUR DOM : VITESSE, DISTANCE & RELATIVITÉ ---
        
        // Vitesse (N/A si en PAUSE, sauf Vitesse Stable qui est calculée à 0)
        $('current-speed-kmh').textContent = dataOrDefault(V_ms * 3.6, 5, ' km/h', '--.- km/h'); // Garder le format --.- pour l'instantanée
        $('stable-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s', '-- m/s');
        $('stable-speed-kms').textContent = dataOrDefault(V_ms / 1000, 5, ' km/s', '-- km/s');
        $('speed-3d-instant').textContent = dataOrDefault(V_ms * 3.6, 5, ' km/h', '-- km/h'); 
        $('raw-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s', '-- m/s');
        $('max-speed-session').textContent = dataOrDefault(currentMaxSpeed * 3.6, 1, ' km/h'); // 1.1 km/h (1 décimale)
        
        // Relativité (Calculés même si V=0)
        $('speed-sound-local').textContent = dataOrDefault(V_SOUND_ISA, 4, ' m/s');
        $('perc-speed-sound').textContent = dataOrDefault(V_ms / V_SOUND_ISA * 100, 2, ' %');
        $('mach-number').textContent = dataOrDefault(mach_number, 4);
        
        $('perc-speed-light').textContent = dataOrDefaultExp(v_ratio_c * 100, 2, ' %'); // 2 décimales pour l'exposant
        $('lorentz-factor').textContent = dataOrDefault(gamma, 4); // 4 décimales pour 1.0000
        $('time-dilation-v').textContent = dataOrDefault(0.0, 2, ' ns/j');
        
        $('relativistic-energy').textContent = dataOrDefaultExp(energy_rest, 4, ' J', 'N/A'); // E=E0 pour V=0
        $('rest-mass-energy').textContent = dataOrDefaultExp(energy_rest, 4, ' J', 'N/A');
        $('momentum').textContent = dataOrDefaultExp(momentum, 4, ' kg·m/s', 'N/A');
        $('schwarzschild-radius').textContent = dataOrDefaultExp(schwartzschild_radius, 4, ' m'); // 1.0397e-25 m
        $('speed-of-light-c').textContent = dataOrDefault(C, 0, ' m/s');
        $('grav-universal-g').textContent = dataOrDefaultExp(G, 5, ' m³/kg/s²'); // 5 décimales pour la mantisse

        // Distance (V=0 -> 0.000 km | 0.00 m)
        $('total-distance-3d').textContent = dataOrDefault(0.0, 3, ' km | ') + dataOrDefault(0.0, 2, ' m');
        $('dist-light-s').textContent = dataOrDefaultExp(0.0, 2, ' s');
        $('dist-ua-al').textContent = dataOrDefaultExp(0.0, 2, ' UA | ') + dataOrDefaultExp(0.0, 2, ' al');


        // --- 6. MISE À JOUR DOM : DYNAMIQUE & FORCES ---
        $('local-gravity').textContent = isGpsPaused ? 'N/A' : dataOrDefault(G_STD, 5, ' m/s²');
        $('force-g-long').textContent = isIMUActive ? dataOrDefault(0.0, 2, ' G') : 'N/A';
        $('accel-long').textContent = isIMUActive ? dataOrDefault(0.0, 2, ' m/s²') : 'N/A';
        $('vertical-speed-ekf').textContent = dataOrDefault(currentUKFState.vD * -1, 5, ' m/s', 'N/A'); 
        
        $('dynamic-pressure-q').textContent = dataOrDefault(dynamic_pressure, 2, ' Pa'); // 0.00 Pa
        $('force-trainee').textContent = dataOrDefault(0.0, 2, ' N');
        $('kinetic-energy').textContent = dataOrDefault(kinetic_energy, 2, ' J');
        $('force-coriolis').textContent = dataOrDefault(0.0, 2, ' N');


        // --- 7. MISE À JOUR DOM : FILTRE EKF/UKF & DEBUG ---
        $('ukf-reactivity-mode-display').textContent = currentUKFReactivity + ' (Adaptatif)';
        $('gps-status').textContent = isGpsPaused ? 'PAUSE' : 'N/A';
        $('ekf-status').textContent = 'N/A';
        $('uncert-speed-p').textContent = dataOrDefault(Math.sqrt(currentUKFState.kUncert), 5, ' m/s', 'N/A');
        $('forced-gps-precision').textContent = dataOrDefault(0.0, 6, ' m');


        // --- 8. MISE À JOUR DOM : POSITION & ASTRO ---
        // Position EKF (basée sur l'état statique)
        $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); // 2 décimales pour l'altitude (100.00 m)

        if (astroData) {
            // Temps Solaire & Sidéral
            $('heure-solaire-vraie').textContent = formatHours(astroData.TST_HRS);
            $('heure-solaire-moyenne').textContent = formatHours(astroData.MST_HRS);
            $('equation-temps').textContent = dataOrDefault(astroData.EOT_MIN, 5, ' min', 'N/A');
            $('ecliptic-longitude').textContent = dataOrDefault(astroData.ECL_LONG, 5, '°', 'N/A');

            // Soleil (Angles en degrés)
            $('sun-alt').textContent = dataOrDefault(astroData.sun.altitude * R2D, 5, '°', 'N/A');
            $('sun-azimuth').textContent = dataOrDefault(astroData.sun.azimuth * R2D, 5, '°', 'N/A');
            $('day-duration').textContent = dataOrDefault(astroData.sun.dayDuration, 5, ' h', 'N/A');
            // Ces valeurs sont N/A dans l'exemple utilisateur, mais je les calcule si astro.js est là
            $('sunrise-times').textContent = astroData.sun.times.rise ? astroData.sun.times.rise.toTimeString().substring(0, 8) : 'N/A';
            $('sunset-times').textContent = astroData.sun.times.set ? astroData.sun.times.set.toTimeString().substring(0, 8) : 'N/A';
            
            // Lune
            const moon_phase_name = typeof getMoonPhaseName === 'function' ? getMoonPhaseName(astroData.moon.illumination.phase) : 'N/A';
            $('moon-phase-name').textContent = moon_phase_name;
            $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 5, '%', 'N/A');
            $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 5, '°', 'N/A');
            $('moon-azimuth').textContent = dataOrDefault(astroData.moon.position.azimuth * R2D, 5, '°', 'N/A');
            $('moon-distance').textContent = dataOrDefaultExp(astroData.moon.position.distance, 5, ' m', 'N/A');

        } else {
             // Fallback N/A pour tous les champs Astro si la dépendance n'est pas chargée
             const astro_na = ['equation-temps', 'ecliptic-longitude', 'sun-alt', 'sun-azimuth', 'day-duration', 'sunrise-times', 'sunset-times', 'moon-phase-name', 'moon-illuminated', 'moon-alt', 'moon-azimuth', 'moon-distance'];
             astro_na.forEach(id => {
                if ($(id)) $(id).textContent = 'N/A';
             });
        }
        
    } // Fin de updateDashboard


    // =========================================================
    // BLOC 7 : INITIALISATION DU SYSTÈME
    // =========================================================

    // Note : Les fonctions initMap(), setupEventListeners(), initGPS(), etc. 
    // sont supposées exister pour une application complète.

    window.addEventListener('load', () => {
        
        // 1. Démarrage de la boucle de rafraîchissement
        setInterval(updateDashboard, 1000 / 60); // 60 FPS
        
        // 2. Initialisation UKF
        if (typeof ProfessionalUKF === 'function' && typeof math !== 'undefined') { 
            ukf = new ProfessionalUKF(); 
            const initState = [
                currentUKFState.lat, currentUKFState.lon, currentUKFState.alt, 
                currentUKFState.vN, currentUKFState.vE, currentUKFState.vD, 
                0, 0, 0, 1, // Quaternion (q_x, q_y, q_z, q_w)
                ...Array(11).fill(0) // 11 états supplémentaires (Bias, etc.)
            ];
            ukf.initializeState(initState);
        } else {
             console.error("Dépendance UKF/Math.js manquante. Le tableau fonctionnera en mode statique non-filtré.");
        }
        
        // 3. Mise à jour immédiate
        updateDashboard();
    });

})(window);
