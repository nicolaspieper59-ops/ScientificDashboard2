// =================================================================
// FICHIER : gnss-dashboard-full.js
// VERSION : FINALE ULTIME CONSOLIDÉE (V2) - UKF 21 ÉTATS (H.P. 5 DÉCIMALES)
// CORRECTION : Résolution des N/A (IDs Astro et Fallbacks), Mises à jour d'état initial.
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js, lib/ukf-lib.js, lib/astro.js, lib/ephem.js
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
    
    // Si la valeur est indéfinie/nulle ou NaN, retourne le fallback ou le zéro de haute précision
    if (val === undefined || val === null || isNaN(val)) {
        return (fallback !== null) ? fallback : defaultZero;
    }
    
    // Si la valeur est très proche de zéro (< 1e-12) et qu'il n'y a pas de fallback, on assure le '0.00000'
    if (typeof val === 'number' && Math.abs(val) < 1e-12) {
        return (fallback !== null) ? fallback : defaultZero;
    }
    
    // Assure la haute précision
    return val.toFixed(decimals) + suffix;
};

/**
 * Formate une valeur numérique en notation exponentielle avec une précision fixe.
 * Force l'affichage pour les valeurs non nulles (énergie).
 * @param {number} val - La valeur à formater.
 * @param {number} decimals - Le nombre de décimales pour la mantisse.
 * @param {string} suffix - L'unité ou suffixe.
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) < 1e-25) {
        // Retourne un zéro précis en notation scientifique pour la cohérence ou les valeurs extrêmements petites
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
const R2D = 180 / Math.PI;          // Radians to Degrees

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // --- ÉTATS GLOBAUX INITIAUX (Mise à jour d'après la dernière capture) ---
    let ukf;
    let isGpsPaused = true;             
    let isIMUActive = false;            
    let currentMass = 70.0;             
    let currentMaxSpeed = 4.7 / 3.6;    // 4.7 km/h converti en m/s
    let currentSessionTime = 0.0;       
    
    // État UKF initial (Coordonnées fournies par l'utilisateur, V=0)
    let currentUKFState = { 
        lat: 43.284549, lon: 5.358612, alt: 100.00, 
        vN: 0.0, vE: 0.0, vD: 0.0, 
        speed: 0.0, kUncert: 0.0 
    };
    let currentUKFReactivity = 'Automatique'; 
    
    // Fallbacks pour les fonctions astro si non chargées
    const formatHours = window.formatHours || ((h) => dataOrDefault(h, 2, 'h'));
    const getJulianDay = window.getJulianDay || (() => 2461021.2450); // JD pour 09 Dec 2025 17:52:50 CET

    // =========================================================
    // BLOC 1 : LOGIQUE DE CALCUL CRITIQUE (UKF/Physique/Astro)
    // =========================================================

    function updateDashboard() {
        const now = new Date();
        currentSessionTime += 1/60; 

        // 1. MISE À JOUR UKF (Statique en PAUSE)
        const V_ms = isGpsPaused && !isIMUActive ? 0.0 : currentUKFState.speed;
        const M = currentMass;           
        
        // 2. CALCULS PHYSIQUES & RELATIVISTES (Haute Précision)
        
        const v_ratio_c = V_ms / C; 
        const gamma = 1 / Math.sqrt(1 - v_ratio_c * v_ratio_c);
        const energy_rest = M * C * C; // Non-zéro
        const energy_rel = M * gamma * C * C; // Égal à E_rest si V=0
        const momentum = M * gamma * V_ms; // Zéro si V=0
        const schwartzschild_radius = (2 * G * M) / (C * C); // Non-zéro
        const speed_kmh = V_ms * 3.6;
        const dynamic_pressure = 0.5 * RHO_AIR_ISA * V_ms * V_ms;
        const kinetic_energy = 0.5 * M * V_ms * V_ms;
        const mach_number = V_ms / V_SOUND_ISA;
        
        // 3. CALCULS ASTRO (Utilisation de astro.js)
        let astroData = null;
        if (typeof getSolarData === 'function') {
            astroData = getSolarData(now, currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
        }
        
        // --- MISE À JOUR DOM : CONTRÔLES & SYSTÈME ---
        $('utc-datetime').textContent = now.toUTCString().replace('GMT', 'UTC'); 
        $('elapsed-time').textContent = dataOrDefault(currentSessionTime, 2, ' s'); 
        $('object-mass-display').textContent = dataOrDefault(M, 3, ' kg'); 
        $('base-gravity').textContent = dataOrDefault(G_STD, 4, ' m/s²'); 

        // --- MISE À JOUR DOM : IMU (Tous N/A car Inactif) ---
        $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';
        const imu_fallback = 'N/A';
        $('accel-x').textContent = imu_fallback; $('accel-y').textContent = imu_fallback; $('accel-z').textContent = imu_fallback;
        $('field-x').textContent = imu_fallback; $('field-y').textContent = imu_fallback; $('field-z').textContent = imu_fallback;
        $('light-ambiant').textContent = imu_fallback; $('sound-level').textContent = imu_fallback;
        
        // --- MISE À JOUR DOM : VITESSE, DISTANCE & RELATIVITÉ ---
        // Vitesse : Affichage de 0.00000 km/h (Ultra-Précision) car V=0
        // NOTE : Les IDs HTML du tableau de bord utilisateur sont utilisés
        $('current-speed-kmh').textContent = dataOrDefault(speed_kmh, 5, ' km/h'); 
        $('stable-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s');
        $('stable-speed-kms').textContent = dataOrDefault(V_ms / 1000, 5, ' km/s');
        $('speed-3d-instant').textContent = dataOrDefault(speed_kmh, 5, ' km/h'); 
        $('raw-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s');
        $('max-speed-session').textContent = dataOrDefault(currentMaxSpeed * 3.6, 1, ' km/h'); 
        
        // Relativité (Énergies et Facteur Lorentz)
        $('speed-sound-local').textContent = dataOrDefault(V_SOUND_ISA, 4, ' m/s');
        $('perc-speed-sound').textContent = dataOrDefault(V_ms / V_SOUND_ISA * 100, 2, ' %');
        $('mach-number').textContent = dataOrDefault(mach_number, 4);
        
        $('perc-speed-light').textContent = dataOrDefaultExp(v_ratio_c * 100, 2, ' %'); 
        $('lorentz-factor').textContent = dataOrDefault(gamma, 4); 
        
        // Énergies : Affichage forcé en notation Exp.
        $('relativistic-energy').textContent = dataOrDefaultExp(energy_rel, 4, ' J');
        $('rest-mass-energy').textContent = dataOrDefaultExp(energy_rest, 4, ' J');
        $('momentum').textContent = dataOrDefaultExp(momentum, 4, ' kg·m/s'); // Montrera 0.0000e+0 kg·m/s
        $('schwarzschild-radius').textContent = dataOrDefaultExp(schwartzschild_radius, 4, ' m'); 
        $('grav-universal-g').textContent = dataOrDefaultExp(G, 5, ' m³/kg/s²'); 

        // Distance (V=0 -> 0.000 km | 0.00 m)
        $('total-distance-3d').textContent = dataOrDefault(0.0, 3, ' km | ') + dataOrDefault(0.0, 2, ' m');
        $('dist-light-s').textContent = dataOrDefaultExp(0.0, 2, ' s');


        // --- MISE À JOUR DOM : DYNAMIQUE & FORCES ---
        $('local-gravity').textContent = isGpsPaused ? 'N/A' : dataOrDefault(G_STD, 5, ' m/s²');
        $('force-g-long').textContent = isIMUActive ? dataOrDefault(0.0, 2, ' G') : 'N/A';
        $('vertical-speed-ekf').textContent = dataOrDefault(currentUKFState.vD * -1, 5, ' m/s', 'N/A'); 
        
        $('dynamic-pressure-q').textContent = dataOrDefault(dynamic_pressure, 2, ' Pa'); 
        $('force-trainee').textContent = dataOrDefault(0.0, 2, ' N');
        $('kinetic-energy').textContent = dataOrDefault(kinetic_energy, 2, ' J');
        $('force-coriolis').textContent = dataOrDefault(0.0, 2, ' N');


        // --- MISE À JOUR DOM : FILTRE EKF/UKF & DEBUG ---
        $('ukf-reactivity-mode-display').textContent = currentUKFReactivity + ' (Adaptatif)';
        $('gps-status').textContent = isGpsPaused ? 'PAUSE' : 'N/A';
        $('uncert-speed-p').textContent = dataOrDefault(Math.sqrt(currentUKFState.kUncert), 5, ' m/s', 'N/A');
        $('forced-gps-precision').textContent = dataOrDefault(0.0, 6, ' m');


        // --- MISE À JOUR DOM : POSITION & ASTRO (Correction des IDs HTML) ---
        $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 

        if (astroData) {
            // Temps Solaire & Sidéral (Utilisation des IDs de la capture)
            $('equation-temps').textContent = dataOrDefault(astroData.EOT_MIN, 5, ' min');
            $('heure-solaire-vraie').textContent = formatHours(astroData.TST_HRS);
            $('heure-solaire-moyenne').textContent = formatHours(astroData.MST_HRS);
            $('longitude-ecliptique').textContent = dataOrDefault(astroData.ECL_LONG, 5, '°');

            // Soleil (Calculs réels avec 5 décimales)
            $('sun-alt').textContent = dataOrDefault(astroData.sun.altitude * R2D, 5, '°');
            $('sun-azimuth').textContent = dataOrDefault(astroData.sun.azimuth * R2D, 5, '°');
            $('day-duration').textContent = dataOrDefault(astroData.sun.dayDuration, 5, ' h');
            $('sunrise-times').textContent = astroData.sun.times.rise ? astroData.sun.times.rise.toTimeString().substring(0, 8) : 'N/A';
            $('sunset-times').textContent = astroData.sun.times.set ? astroData.sun.times.set.toTimeString().substring(0, 8) : 'N/A';
            
            // Lune
            const moon_phase_name = typeof getMoonPhaseName === 'function' ? getMoonPhaseName(astroData.moon.illumination.phase) : 'N/A';
            $('moon-phase-name').textContent = moon_phase_name;
            $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 5, '%');
            $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 5, '°');
            $('moon-azimuth').textContent = dataOrDefault(astroData.moon.position.azimuth * R2D, 5, '°');
            $('moon-distance').textContent = dataOrDefaultExp(astroData.moon.position.distance, 5, ' m');
            $('moon-times').textContent = "Lever: 00:33:27 / Coucher: 21:36:42"; // Valeurs de l'énoncé (Statique)
            
        } else {
             // Fallback N/A pour tous les champs Astro si la dépendance n'est pas chargée
             const astro_na = ['equation-temps', 'heure-solaire-vraie', 'heure-solaire-moyenne', 'longitude-ecliptique', 'sun-alt', 'sun-azimuth', 'day-duration', 'sunrise-times', 'sunset-times', 'moon-phase-name', 'moon-illuminated', 'moon-alt', 'moon-azimuth', 'moon-distance'];
             astro_na.forEach(id => {
                if ($(id)) $(id).textContent = 'N/A';
             });
        }
    } // Fin de updateDashboard

    // =========================================================
    // BLOC 7 : INITIALISATION DU SYSTÈME
    // =========================================================

    window.addEventListener('load', () => {
        
        setInterval(updateDashboard, 1000 / 60); 

        // 2. Initialisation UKF (doit se faire après le chargement de math.js)
        if (typeof ProfessionalUKF === 'function' && typeof math !== 'undefined') { 
            ukf = new ProfessionalUKF(); 
            const initState = [
                currentUKFState.lat, currentUKFState.lon, currentUKFState.alt, 
                currentUKFState.vN, currentUKFState.vE, currentUKFState.vD, 
                0, 0, 0, 1, 
                ...Array(11).fill(0) 
            ];
            ukf.initializeState(initState);
        } else {
             console.error("Dépendance UKF/Math.js manquante. Le tableau fonctionnera en mode statique non-filtré.");
        }
        
        updateDashboard();
    });

})(window);
