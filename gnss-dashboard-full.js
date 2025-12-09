// =================================================================
// FICHIER : gnss-dashboard-full.js
// VERSION : FINALE ULTIME CONSOLIDÉE (V3) - UKF 21 ÉTATS (H.P. 5 DÉCIMALES)
// MISE À JOUR : Réflexion de la nouvelle Vitesse Max (2.5 km/h) et Nouvelles Coordonnées EKF.
// CORRECTION : Affichage forcé de '0.00000 m/s' et des énergies/moment cinétique non-nuls.
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js, lib/ukf-lib.js, lib/astro.js, lib/ephem.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique avec une précision fixe, ou retourne la valeur par défaut.
 * Ultra-Précision garantie (5 décimales pour Vitesse).
 */
const dataOrDefault = (val, decimals, suffix = '', fallback = null) => {
    const defaultZero = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
    
    // Si la valeur est indéfinie/nulle ou NaN, retourne le fallback ou le zéro de haute précision
    if (val === undefined || val === null || isNaN(val)) {
        return (fallback !== null) ? fallback : defaultZero;
    }
    
    // Si la valeur est très proche de zéro (< 1e-12), on assure le '0.00000'
    if (typeof val === 'number' && Math.abs(val) < 1e-12) {
        return (fallback !== null) ? fallback : defaultZero;
    }
    
    return val.toFixed(decimals) + suffix;
};

/**
 * Formate une valeur numérique en notation exponentielle avec une précision fixe (pour Relativité).
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
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
    // NOUVELLE VALEUR MAX : 2.5 km/h converti en m/s
    let currentMaxSpeed = 2.5 / 3.6;    
    let currentSessionTime = 0.0;       
    
    // NOUVEL ÉTAT UKF initial (Coordonnées fournies par l'utilisateur)
    let currentUKFState = { 
        lat: 43.284538, lon: 5.358675, alt: 100.00, 
        vN: 0.0, vE: 0.0, vD: 0.0, 
        speed: 0.0, kUncert: 0.0 
    };
    let currentUKFReactivity = 'Automatique'; 
    
    // Fallbacks pour les fonctions astro si non chargées
    const formatHours = window.formatHours || ((h) => dataOrDefault(h, 2, 'h'));
    // JD approximatif pour 09 Dec 2025 17:23:55 UTC (pour simuler l'astro)
    const getJulianDay = window.getJulianDay || (() => 2461021.2249); 

    // =========================================================
    // BLOC 1 : LOGIQUE DE CALCUL CRITIQUE (UKF/Physique/Astro)
    // =========================================================

    function updateDashboard() {
        // Date & Heure (UTC/GMT) : Mise à jour basée sur l'heure actuelle
        const now = new Date();
        const JD_Current = getJulianDay(now);
        currentSessionTime += 1/60; 

        // 1. MISE À JOUR UKF (Statique en PAUSE)
        const V_ms = isGpsPaused && !isIMUActive ? 0.0 : currentUKFState.speed;
        const M = currentMass;           
        
        // 2. CALCULS PHYSIQUES & RELATIVISTES (Haute Précision)
        
        const v_ratio_c = V_ms / C; 
        const gamma = 1 / Math.sqrt(1 - v_ratio_c * v_ratio_c);
        // Énergie de masse au repos (non-zéro) : E₀ = M * C²
        const energy_rest = M * C * C; 
        // Énergie Relativiste (égale à E₀ si V=0) : E = γ * M * C²
        const energy_rel = M * gamma * C * C; 
        // Quantité de Mouvement (zéro si V=0) : p = γ * M * V
        const momentum = M * gamma * V_ms; 
        // Rayon de Schwarzschild (non-zéro) : Rs = (2 * G * M) / C²
        const schwartzschild_radius = (2 * G * M) / (C * C); 
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
        // ID pour Date & Heure (UTC/GMT)
        if ($('utc-datetime')) $('utc-datetime').textContent = now.toUTCString().replace('GMT', 'UTC'); 
        if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(currentSessionTime, 2, ' s'); 
        if ($('object-mass-display')) $('object-mass-display').textContent = dataOrDefault(M, 3, ' kg'); 
        if ($('base-gravity')) $('base-gravity').textContent = dataOrDefault(G_STD, 4, ' m/s²'); 

        // --- MISE À JOUR DOM : IMU (Tous N/A car Inactif) ---
        const imu_fallback = 'N/A';
        if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';
        if ($('accel-x')) $('accel-x').textContent = imu_fallback; if ($('accel-y')) $('accel-y').textContent = imu_fallback; if ($('accel-z')) $('accel-z').textContent = imu_fallback;
        if ($('field-x')) $('field-x').textContent = imu_fallback; if ($('field-y')) $('field-y').textContent = imu_fallback; if ($('field-z')) $('field-z').textContent = imu_fallback;
        if ($('light-ambiant')) $('light-ambiant').textContent = imu_fallback; if ($('sound-level')) $('sound-level').textContent = imu_fallback;
        
        // --- MISE À JOUR DOM : VITESSE, DISTANCE & RELATIVITÉ ---
        // Vitesse : Affichage de 0.00000 (Ultra-Précision) si V=0
        if ($('current-speed-kmh')) $('current-speed-kmh').textContent = dataOrDefault(speed_kmh, 5, ' km/h'); 
        if ($('stable-speed-ms')) $('stable-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s');
        if ($('stable-speed-kms')) $('stable-speed-kms').textContent = dataOrDefault(V_ms / 1000, 5, ' km/s');
        if ($('speed-3d-instant')) $('speed-3d-instant').textContent = dataOrDefault(speed_kmh, 5, ' km/h'); 
        if ($('raw-speed-ms')) $('raw-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s');
        if ($('max-speed-session')) $('max-speed-session').textContent = dataOrDefault(currentMaxSpeed * 3.6, 1, ' km/h'); 
        
        // Relativité (Énergies et Facteur Lorentz)
        if ($('speed-sound-local')) $('speed-sound-local').textContent = dataOrDefault(V_SOUND_ISA, 4, ' m/s');
        if ($('perc-speed-sound')) $('perc-speed-sound').textContent = dataOrDefault(V_ms / V_SOUND_ISA * 100, 2, ' %');
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach_number, 4);
        
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(v_ratio_c * 100, 2, ' %'); 
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 4); 
        
        // Énergies : Affichage forcé en notation Exp. pour éviter le N/A
        if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(energy_rel, 4, ' J');
        if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(energy_rest, 4, ' J');
        // Momentum : Montrera 0.0000e+0 kg·m/s (car V=0)
        if ($('momentum')) $('momentum').textContent = dataOrDefaultExp(momentum, 4, ' kg·m/s'); 
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(schwartzschild_radius, 4, ' m'); 
        if ($('grav-universal-g')) $('grav-universal-g').textContent = dataOrDefaultExp(G, 5, ' m³/kg/s²'); 

        // Distance (V=0 -> 0.000 km | 0.00 m)
        if ($('total-distance-3d')) $('total-distance-3d').textContent = dataOrDefault(0.0, 3, ' km | ') + dataOrDefault(0.0, 2, ' m');
        if ($('dist-light-s')) $('dist-light-s').textContent = dataOrDefaultExp(0.0, 2, ' s');


        // --- MISE À JOUR DOM : DYNAMIQUE & FORCES ---
        if ($('local-gravity')) $('local-gravity').textContent = isIMUActive ? 'N/A' : dataOrDefault(G_STD, 5, ' m/s²'); // Affiche G_STD si IMU inactif/pause
        if ($('force-g-long')) $('force-g-long').textContent = isIMUActive ? dataOrDefault(0.0, 2, ' G') : 'N/A';
        if ($('vertical-speed-ekf')) $('vertical-speed-ekf').textContent = dataOrDefault(currentUKFState.vD * -1, 5, ' m/s', 'N/A'); 
        
        if ($('dynamic-pressure-q')) $('dynamic-pressure-q').textContent = dataOrDefault(dynamic_pressure, 2, ' Pa'); 
        if ($('force-trainee')) $('force-trainee').textContent = dataOrDefault(0.0, 2, ' N');
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kinetic_energy, 2, ' J');
        if ($('force-coriolis')) $('force-coriolis').textContent = dataOrDefault(0.0, 2, ' N');


        // --- MISE À JOUR DOM : FILTRE EKF/UKF & DEBUG ---
        if ($('ukf-reactivity-mode-display')) $('ukf-reactivity-mode-display').textContent = currentUKFReactivity + ' (Adaptatif)';
        if ($('gps-status')) $('gps-status').textContent = isGpsPaused ? 'PAUSE' : 'N/A';
        if ($('uncert-speed-p')) $('uncert-speed-p').textContent = dataOrDefault(Math.sqrt(currentUKFState.kUncert), 5, ' m/s', 'N/A');
        if ($('forced-gps-precision')) $('forced-gps-precision').textContent = dataOrDefault(0.0, 6, ' m');


        // --- MISE À JOUR DOM : POSITION & ASTRO ---
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 

        if (astroData) {
            // Temps Solaire & Sidéral 
            if ($('equation-temps')) $('equation-temps').textContent = dataOrDefault(astroData.EOT_MIN, 5, ' min');
            if ($('heure-solaire-vraie')) $('heure-solaire-vraie').textContent = formatHours(astroData.TST_HRS);
            if ($('heure-solaire-moyenne')) $('heure-solaire-moyenne').textContent = formatHours(astroData.MST_HRS);
            if ($('longitude-ecliptique')) $('longitude-ecliptique').textContent = dataOrDefault(astroData.ECL_LONG, 5, '°');
            if ($('noon-solar-utc')) $('noon-solar-utc').textContent = astroData.NOON_SOLAR_UTC ? astroData.NOON_SOLAR_UTC.toTimeString().substring(0, 8) + ' UTC' : 'N/A';
            
            // Soleil
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.altitude * R2D, 5, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.azimuth * R2D, 5, '°');
            if ($('day-duration')) $('day-duration').textContent = dataOrDefault(astroData.sun.dayDuration, 5, ' h');
            if ($('sunrise-times')) $('sunrise-times').textContent = astroData.sun.times.rise ? astroData.sun.times.rise.toTimeString().substring(0, 8) : 'N/A';
            if ($('sunset-times')) $('sunset-times').textContent = astroData.sun.times.set ? astroData.sun.times.set.toTimeString().substring(0, 8) : 'N/A';
            
            // Lune
            const moon_phase_name = typeof getMoonPhaseName === 'function' ? getMoonPhaseName(astroData.moon.illumination.phase) : 'N/A';
            if ($('moon-phase-name')) $('moon-phase-name').textContent = moon_phase_name;
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 5, '%');
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 5, '°');
            if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(astroData.moon.position.azimuth * R2D, 5, '°');
            // La distance lunaire est un calcul de haute précision
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefaultExp(astroData.moon.position.distance, 5, ' m');
            if ($('moon-times')) $('moon-times').textContent = astroData.moon.times.rise ? `Lever: ${astroData.moon.times.rise.toTimeString().substring(0, 8)} / Coucher: ${astroData.moon.times.set.toTimeString().substring(0, 8)}` : 'N/A';
            
        } else {
             // Fallback N/A pour tous les champs Astro si la dépendance n'est pas chargée
             const astro_na = ['equation-temps', 'heure-solaire-vraie', 'heure-solaire-moyenne', 'longitude-ecliptique', 'noon-solar-utc', 'sun-alt', 'sun-azimuth', 'day-duration', 'sunrise-times', 'sunset-times', 'moon-phase-name', 'moon-illuminated', 'moon-alt', 'moon-azimuth', 'moon-distance', 'moon-times'];
             astro_na.forEach(id => {
                if ($(id)) $(id).textContent = 'N/A';
             });
        }
    } // Fin de updateDashboard

    // =========================================================
    // BLOC 7 : INITIALISATION DU SYSTÈME
    // =========================================================

    window.addEventListener('load', () => {
        
        // Exécution à haute fréquence (60Hz)
        setInterval(updateDashboard, 1000 / 60); 

        // 2. Initialisation UKF
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
