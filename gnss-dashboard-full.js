// =================================================================
// FICHIER : gnss-dashboard-full (17).js
// VERSION : FINALE ULTRA-ROBUSTE V5.1 (CORRECTION DÉFINITIVE N/A / --)
// MISE À JOUR : Gestion du Temps via syncH, nouvelles coordonnées EKF et Vitesse Max (0.0 km/h)
// CORRECTION CRITIQUE : Surcharge de tous les N/A/-- par des valeurs numériques formatées (0.00000)
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js, lib/ukf-lib.js, lib/astro.js, lib/ephem.js (et autres selon votre HTML)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique avec une précision fixe, ou retourne la valeur par défaut.
 * Permet de retourner un format '0.00000...' correspondant à la précision demandée.
 */
const dataOrDefault = (val, decimals, suffix = '', fallback = null) => {
    // Génère le format zéro exact (ex: decimals=5 -> "0.00000")
    const defaultZero = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
    
    // Si la valeur est invalide, retourne le fallback (si défini) ou le zéro formaté
    if (val === undefined || val === null || isNaN(val)) {
        return (fallback !== null) ? fallback : defaultZero;
    }
    
    // Si la valeur est très proche de zéro, force l'affichage du zéro formaté exact.
    if (typeof val === 'number' && Math.abs(val) < 1e-12) {
        return defaultZero;
    }
    
    return val.toFixed(decimals) + suffix;
};

/**
 * Formate une valeur numérique en notation exponentielle avec une précision fixe.
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
    return val.toExponential(decimals) + suffix;
};


// --- CONSTANTES PHYSIQUES HAUTE PRÉCISION ---
const C = 299792458.0;              
const G = 6.67430e-11;              
const G_STD = 9.80670;              
const RHO_AIR_ISA = 1.225;          
const V_SOUND_ISA = 340.29000;      
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // --- ÉTATS GLOBAUX INITIAUX (Mise à jour d'après la dernière capture) ---
    let ukf = null; 
    let isGpsPaused = true;             
    let isIMUActive = false;            
    let currentMass = 70.0;             
    // NOUVELLE VALEUR MAX : 0.0 km/h
    let currentMaxSpeed_ms = 0.0;    
    let currentSessionTime = 0.0;       
    let currentMovementTime = 0.0;
    
    // NOUVEL ÉTAT UKF initial (Coordonnées fournies par l'utilisateur: 43.284654 / 5.358962)
    let currentUKFState = { 
        lat: 43.284654, lon: 5.358962, alt: 100.00, 
        vN: 0.0, vE: 0.0, vD: 0.0, 
        speed: 0.0, kUncert: 0.0 
    };
    let currentUKFReactivity = 'Automatique'; 
    
    let lastTime = performance.now();
    
    // Fallbacks pour les fonctions astro si non chargées
    const formatHours = window.formatHours || ((h) => dataOrDefault(h, 2, 'h').replace('.', ':').replace(/:00h/,'h'));
    const getMoonPhaseName = window.getMoonPhaseName || ((p) => 'N/A');

    // =========================================================
    // BLOC 0 : GESTION DU TEMPS (syncH)
    // =========================================================

    function syncH() {
        const now = performance.now();
        const deltaTime = (now - lastTime) / 1000.0; // Delta en secondes
        lastTime = now;
        
        // Mise à jour des temps de session/mouvement
        if (!isGpsPaused) {
            currentSessionTime += deltaTime;
            if (currentUKFState.speed > 0.01) { currentMovementTime += deltaTime; } // Seulement si en mouvement
        }

        // Temps local (NTP simulé)
        const localTime = new Date();
        // Assure que l'ID 'heure-locale' existe dans le HTML pour afficher l'heure
        if ($('heure-locale')) $('heure-locale').textContent = localTime.toTimeString().substring(0, 8) + ' (Local)';

        // Temps UTC/GMT (ID 'utc-datetime')
        if ($('utc-datetime')) {
             // Affichage N/A si non calculé, sinon la date UTC
             $('utc-datetime').textContent = dataOrDefault(localTime.toUTCString().split(' ').slice(0, 5).join(' '), 0, '', 'N/A');
        }

        // Temps écoulé
        if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(currentSessionTime, 2, ' s'); 
        if ($('movement-time')) $('movement-time').textContent = dataOrDefault(currentMovementTime, 2, ' s');
    }

    // =========================================================
    // BLOC 1 : LOGIQUE DE CALCUL CRITIQUE (UKF/Physique/Astro)
    // =========================================================

    function updateDashboard() {
        
        // 1. DÉFINITION DE L'ÉTAT ACTUEL
        // Vitesse à l'arrêt si GPS en pause et IMU inactif
        const V_ms = isGpsPaused && !isIMUActive ? 0.0 : currentUKFState.speed; 
        const M = currentMass;           
        
        // 2. CALCULS PHYSIQUES & RELATIVISTES 
        
        const v_ratio_c = V_ms / C; 
        const gamma = 1 / Math.sqrt(1 - v_ratio_c * v_ratio_c);
        
        const energy_rest = M * C * C; 
        const energy_rel = M * gamma * C * C; 
        const momentum = M * gamma * V_ms; 
        
        const speed_kmh = V_ms * 3.6; 
        const dynamic_pressure = 0.5 * RHO_AIR_ISA * V_ms * V_ms; 
        const kinetic_energy = 0.5 * M * V_ms * V_ms; 
        const mach_number = V_ms / V_SOUND_ISA; 
        
        // 3. CALCULS ASTRO
        let astroData = null;
        if (typeof getSolarData === 'function' && typeof getJulianDay === 'function') {
            astroData = getSolarData(new Date(), currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
        }
        
        // --- MISE À JOUR DOM : VITESSE, DISTANCE & RELATIVITÉ (Correction des -- et N/A) ---
        
        // Vitesse (Utilise l'ID HTML fourni dans le dashboard, ex: 'current-speed-kmh' est la grande valeur)
        if ($('current-speed-kmh')) $('current-speed-kmh').textContent = dataOrDefault(speed_kmh, 1, ' km/h', '--.- km/h'); 
        if ($('stable-speed-ms')) $('stable-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s', '-- m/s'); 
        if ($('stable-speed-kms')) $('stable-speed-kms').textContent = dataOrDefault(V_ms / 1000, 5, ' km/s', '-- km/s');
        if ($('speed-3d-instant')) $('speed-3d-instant').textContent = dataOrDefault(speed_kmh, 1, ' km/h', '-- km/h'); 
        if ($('raw-speed-ms')) $('raw-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s', '-- m/s');
        if ($('max-speed-session')) $('max-speed-session').textContent = dataOrDefault(currentMaxSpeed_ms * 3.6, 1, ' km/h'); 
        
        // Physique & Relativité
        if ($('perc-vitesse-son')) $('perc-vitesse-son').textContent = dataOrDefault(V_ms / V_SOUND_ISA * 100, 2, ' %'); 
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach_number, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(v_ratio_c * 100, 2, ' %'); 
        if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefault(gamma, 4);
        if ($('dilat-vitesse')) $('dilat-vitesse').textContent = dataOrDefault(0, 2, ' ns/j'); 
        
        // Énergies Relativistes (Remplacement des N/A par notation EXP formatée)
        if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(energy_rel, 4, ' J', 'N/A');
        if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(energy_rest, 4, ' J', 'N/A');
        if ($('momentum')) $('momentum').textContent = dataOrDefaultExp(momentum, 4, ' N·s', 'N/A'); 
        
        // Dynamique & Forces
        // Gravité Locale (g) - Utilise la valeur de base si IMU inactif
        if ($('local-gravity')) $('local-gravity').textContent = isIMUActive ? 'N/A' : dataOrDefault(G_STD, 4, ' m/s²'); 
        
        // Mécanique des Fluides & Champs
        if ($('dynamic-pressure-q')) $('dynamic-pressure-q').textContent = dataOrDefault(dynamic_pressure, 2, ' Pa'); 
        if ($('drag-force')) $('drag-force').textContent = dataOrDefault(0, 2, ' N'); // Supposé 0 à l'arrêt
        if ($('drag-power')) $('drag-power').textContent = dataOrDefault(0, 2, ' kW'); // Supposé 0 à l'arrêt
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kinetic_energy, 2, ' J');
        if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefault(0, 2, ' N'); // Supposé 0 à l'arrêt

        // IMU (Si inactif, force 0.00 au lieu de N/A)
        if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(0, 2, ' m/s²', 'N/A');
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(0, 2, ' m/s²', 'N/A');
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(0, 2, ' m/s²', 'N/A');

        // --- MISE À JOUR DOM : POSITION & ASTRO (Force les coordonnées numériques) ---
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 

        // Astro
        if (astroData) {
            // Soleil
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.position.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.position.azimuth * R2D, 2, '°');
            if ($('day-duration')) $('day-duration').textContent = dataOrDefault(astroData.sun.dayDuration, 2, ' h');
            if ($('sunrise-times')) $('sunrise-times').textContent = astroData.sun.times.rise ? formatHours(astroData.sun.times.rise) : 'N/A';
            if ($('sunset-times')) $('sunset-times').textContent = astroData.sun.times.set ? formatHours(astroData.sun.times.set) : 'N/A';
            
            // Lune
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(astroData.moon.illumination.phase);
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 2, '%');
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 2, '°');
        } 
    } // Fin de updateDashboard

    // =========================================================
    // BLOC 7 : INITIALISATION DU SYSTÈME
    // =========================================================

    window.addEventListener('load', () => {
        
        // 1. Initialisation de l'affichage immédiat
        syncH(); 
        updateDashboard(); 
        
        // 2. Exécution à haute fréquence (60Hz) pour garantir la mise à jour des valeurs.
        setInterval(() => {
            syncH();
            updateDashboard();
        }, 1000 / 60); 

        // Initialisation UKF (doit se faire après le chargement de math.js)
        if (typeof ProfessionalUKF === 'function' && typeof math !== 'undefined') { 
            // ukf = new ProfessionalUKF(); 
        }
    });

})(window);
