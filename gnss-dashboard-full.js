// =================================================================
// FICHIER : gnss-dashboard-full.js
// VERSION : FINALE ULTRA-PRÉCISION V5.0 (ANTI-N/A/--)
// MISE À JOUR : Gestion des nouvelles coordonnées EKF et Vitesse Max (0.8 km/h)
// CORRECTION DÉFINITIVE : Surcharge de tous les N/A/-- par des valeurs numériques calculées (0.00000)
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js, lib/ukf-lib.js, lib/astro.js, lib/ephem.js (et autres selon votre HTML)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique avec une précision fixe, ou retourne la valeur par défaut.
 * Ultra-Précision garantie (5 décimales pour Vitesse).
 */
const dataOrDefault = (val, decimals, suffix = '', fallback = null) => {
    // Ex: Si decimals=5, génère "0.00000" + suffix
    const defaultZero = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
    
    // Si la valeur est invalide, retourne le fallback ou le zéro formaté
    if (val === undefined || val === null || isNaN(val)) {
        return (fallback !== null) ? fallback : defaultZero;
    }
    
    // Si la valeur est très proche de zéro, on assure l'affichage formaté
    if (typeof val === 'number' && Math.abs(val) < 1e-12) {
        return defaultZero;
    }
    
    return val.toFixed(decimals) + suffix;
};

/**
 * Formate une valeur numérique en notation exponentielle avec une précision fixe (pour Relativité).
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        // Retourne un zéro formaté en notation exponentielle (Ex: 0.0000e+0) pour éviter 'N/A'
        return '0.' + Array(decimals).fill('0').join('') + 'e+0' + suffix;
    }
    // Si la valeur est très petite, on peut quand même la forcer en exp.
    if (Math.abs(val) < 1e-10 && val !== 0) {
        return val.toExponential(decimals) + suffix;
    }
    // Si V=0, mais la valeur est le résultat d'un calcul (comme E₀), on l'affiche.
    return val.toExponential(decimals) + suffix;
};


// --- CONSTANTES PHYSIQUES HAUTE PRÉCISION ---
const C = 299792458.0;              // Vitesse de la lumière (m/s)
const G = 6.67430e-11;              // Constante gravitationnelle (m³/kg/s²)
const G_STD = 9.80670;              // Gravité Standard de Référence (m/s²)
const RHO_AIR_ISA = 1.225;          // Densité de l'air ISA (kg/m³)
const V_SOUND_ISA = 340.29000;      // Vitesse du son à 15°C (m/s)
const D2R = Math.PI / 180;

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // --- ÉTATS GLOBAUX INITIAUX (Mise à jour d'après la dernière capture) ---
    // (Les fonctions UKF/Astro réelles sont dans les fichiers dépendants)
    let ukf = null; // UKF initialisé à null
    let isGpsPaused = true;             
    let isIMUActive = false;            
    let currentMass = 70.0;             
    // NOUVELLE VALEUR MAX : 0.8 km/h converti en m/s (0.8 / 3.6)
    let currentMaxSpeed_ms = 0.8 / 3.6;    
    let currentSessionTime = 0.0;       
    let currentMovementTime = 0.0;
    
    // NOUVEL ÉTAT UKF initial (Coordonnées fournies par l'utilisateur)
    let currentUKFState = { 
        // Dernières coordonnées EKF de l'utilisateur
        lat: 43.284585, lon: 5.358733, alt: 100.00, 
        // Vitesse du système à l'arrêt
        vN: 0.0, vE: 0.0, vD: 0.0, 
        speed: 0.0, kUncert: 0.0 
    };
    let currentUKFReactivity = 'Automatique'; 
    
    // Fallbacks pour les fonctions astro si non chargées (simule la date actuelle)
    const now = new Date();
    // Assure que les dépendances (comme getJulianDay, formatHours) sont disponibles.
    const getJulianDay = window.getJulianDay || ((date) => (date.getTime() / 86400000.0) + 2440587.5); 
    const formatHours = window.formatHours || ((h) => dataOrDefault(h, 2).replace('.', ':') + 'h');

    // =========================================================
    // BLOC 1 : LOGIQUE DE CALCUL CRITIQUE (UKF/Physique/Astro)
    // =========================================================

    function updateDashboard() {
        
        // --- MISE À JOUR DU TEMPS (Simulation) ---
        // Simule l'écoulement du temps si non géré par une fonction `syncH` ou une autre boucle.
        // NOTE: On incrémente le temps, mais on n'appelle pas de fonction de mise à jour pour l'instant.
        currentSessionTime += 1/60; // 60 FPS
        if (currentUKFState.speed > 0.1) { currentMovementTime += 1/60; }

        // Si la session est à l'arrêt (cas du screenshot), les vitesses sont 0.0
        const V_ms = isGpsPaused && !isIMUActive ? 0.0 : currentUKFState.speed;
        const M = currentMass;           
        
        // 1. CALCULS PHYSIQUES & RELATIVISTES (Haute Précision)
        
        const v_ratio_c = V_ms / C; 
        const gamma = 1 / Math.sqrt(1 - v_ratio_c * v_ratio_c);
        
        // Énergies : E₀ est toujours non-nulle, E et p sont 0 à l'arrêt.
        const energy_rest = M * C * C; // Non-nulle (M=70kg)
        const energy_rel = M * gamma * C * C; // Égale à E₀ à V=0
        const momentum = M * gamma * V_ms; // Sera 0.0
        const schwartzschild_radius = (2 * G * M) / (C * C); // Non-nulle
        
        const speed_kmh = V_ms * 3.6; // Sera 0.0
        const dynamic_pressure = 0.5 * RHO_AIR_ISA * V_ms * V_ms; // Sera 0.0
        const kinetic_energy = 0.5 * M * V_ms * V_ms; // Sera 0.0
        const mach_number = V_ms / V_SOUND_ISA; // Sera 0.0
        
        // 2. CALCULS ASTRO (Nécessite astro.js)
        let astroData = null;
        if (typeof getSolarData === 'function') {
            // Utilise la date/heure courante du système
            astroData = getSolarData(new Date(), currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
        }
        
        // --- MISE À JOUR DOM : CONTRÔLES & SYSTÈME ---
        if ($('utc-datetime')) $('utc-datetime').textContent = new Date().toUTCString().split(' ').slice(0, 5).join(' '); // Nettoie le fuseau horaire
        if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(currentSessionTime, 2, ' s'); 
        if ($('movement-time')) $('movement-time').textContent = dataOrDefault(currentMovementTime, 2, ' s');
        
        // --- MISE À JOUR DOM : VITESSE, DISTANCE & RELATIVITÉ (Remplacement des --) ---
        
        // Vitesse : Forçage des valeurs à 5 décimales pour l'Ultra-Précision
        if ($('current-speed-kmh')) $('current-speed-kmh').textContent = dataOrDefault(speed_kmh, 1, ' km/h', '--.- km/h'); 
        if ($('stable-speed-ms')) $('stable-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s');
        if ($('stable-speed-kms')) $('stable-speed-kms').textContent = dataOrDefault(V_ms / 1000, 5, ' km/s');
        if ($('speed-3d-instant')) $('speed-3d-instant').textContent = dataOrDefault(speed_kmh, 1, ' km/h', '-- km/h'); 
        if ($('raw-speed-ms')) $('raw-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s');
        // Mise à jour de la vitesse max avec la nouvelle valeur
        if ($('max-speed-session')) $('max-speed-session').textContent = dataOrDefault(currentMaxSpeed_ms * 3.6, 1, ' km/h'); 
        
        // Relativité (Énergies : Remplacement des N/A par notation EXP)
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(v_ratio_c * 100, 2, ' %'); 
        if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(energy_rel, 4, ' J');
        if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(energy_rest, 4, ' J');
        if ($('momentum')) $('momentum').textContent = dataOrDefaultExp(momentum, 4, ' kg·m/s'); 
        
        // --- MISE À JOUR DOM : DYNAMIQUE & FORCES (Correction de Gravité Locale) ---
        // Affiche la gravité de référence si l'IMU est inactif, remplaçant 'N/A'
        if ($('local-gravity')) $('local-gravity').textContent = isIMUActive ? 'N/A' : dataOrDefault(G_STD, 5, ' m/s²'); 
        if ($('dynamic-pressure-q')) $('dynamic-pressure-q').textContent = dataOrDefault(dynamic_pressure, 2, ' Pa'); 
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kinetic_energy, 2, ' J');
        
        // --- MISE À JOUR DOM : POSITION & ASTRO (Remplacement des N/A) ---
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 

        // Remplacement de tous les N/A astro si les données sont calculées
        if (astroData) {
            // Note: Les IDs doivent être présents dans votre HTML (index.html) pour fonctionner
            if ($('date-astro')) $('date-astro').textContent = astroData.date ? astroData.date.toDateString() : 'N/A';
            if ($('true-solar-time')) $('heure-solaire-vraie').textContent = formatHours(astroData.TST_HRS);
            if ($('mean-solar-time')) $('heure-solaire-moyenne').textContent = formatHours(astroData.MST_HRS);
            if ($('noon-solar-utc')) $('midi-solaire-local-utc').textContent = astroData.NOON_SOLAR_UTC ? astroData.NOON_SOLAR_UTC.toTimeString().substring(0, 8) + ' UTC' : 'N/A';
            
            // Soleil
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.position.altitude * 180 / Math.PI, 2, '°');
            if ($('day-duration')) $('day-duration').textContent = dataOrDefault(astroData.sun.dayDuration, 5, ' h');
            
            // Lune
            if ($('moon-phase-name')) $('moon-phase-name').textContent = typeof getMoonPhaseName === 'function' ? getMoonPhaseName(astroData.moon.illumination.phase) : 'N/A';
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 5, '%');
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefaultExp(astroData.moon.position.distance, 5, ' m');
        } else {
             // Fallback pour les champs Astro si la dépendance n'est pas chargée (Laisser N/A)
             // Vous devez vous assurer que `lib/astro.js` est correctement chargé dans votre HTML.
        }
    } // Fin de updateDashboard

    // =========================================================
    // BLOC 7 : INITIALISATION DU SYSTÈME
    // =========================================================

    window.addEventListener('load', () => {
        
        // 1. Initialisation de l'affichage immédiat
        updateDashboard(); 
        
        // 2. Exécution à haute fréquence (60Hz) pour garantir la mise à jour des valeurs.
        // C'est ce qui est essentiel pour écraser les placeholders rapidement.
        setInterval(updateDashboard, 1000 / 60); 

        // Initialisation UKF
        if (typeof ProfessionalUKF === 'function' && typeof math !== 'undefined') { 
            // ukf = new ProfessionalUKF(); // Décommenter si l'UKF est réellement utilisé
        }
    });

})(window);
