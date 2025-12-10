// =================================================================
// FICHIER : gnss-dashboard-full.js (V7.3 - FIX CRITIQUE GRAVITÉ/CAPTEURS)
// VERSION : FINALE ULTRA-ROBUSTE V7.3 (Correction Gravité et Initialisation Capteurs)
// DÉPENDANCE CRITIQUE: window.getGravity DOIT être défini dans lib/ukf-lib.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique avec une précision fixe, ou retourne la valeur par défaut.
 * Utilise la virgule (,) comme séparateur décimal.
 */
const dataOrDefault = (val, decimals, suffix = '', fallback = null, forceZero = true) => {
    if (val === 'N/A') return 'N/A'; 
    if (val === undefined || val === null || isNaN(val) || (typeof val === 'number' && Math.abs(val) < 1e-18)) {
        if (fallback !== null) return fallback;
        if (forceZero) {
            const zeroFormat = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
            return zeroFormat.replace('.', ',');
        }
        return 'N/A';
    }
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

/**
 * Formate une valeur numérique en notation exponentielle avec une précision fixe, ou retourne 'N/A'.
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) < 1e-30) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals).replace('.', ',') + suffix;
};


// --- CONSTANTES PHYSIQUES HAUTE PRÉCISION ---
const C = 299792458.0;              
const G = 6.67430e-11;              
const G_STD = 9.8067;               
const RHO_AIR_ISA = 1.225;          
const V_SOUND_ISA = 340.2900;       
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180; // Ajout de D2R pour la conversion des coordonnées

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // --- ÉTATS GLOBAUX INITIAUX ---
    let ukf = null; 
    let isGpsPaused = true; // 🟢 FIX: Mode PAUSE GPS par défaut           
    let isIMUActive = false;            
    let currentMass = 70.0;             
    
    let currentMaxSpeed_ms = 0.0 / 3.6;    
    let currentSessionTime = 0.00;       
    let currentMovementTime = 0.00;
    
    // État UKF initial (utilisé pour les calculs astro)
    let currentUKFState = { 
        lat: 43.284572, lon: 5.358710, alt: 100.00, 
        vN: 0.0, vE: 0.0, vD: 0.0, 
        speed: 0.0, kUncert: 0.0 
    };
    let currentUKFReactivity = 'Automatique'; 
    
    let lastTime = performance.now();
    
    // --- VÉRIFICATION ET FALLBACKS DES DÉPENDANCES ASTRO ---
    const formatHours = window.formatHours || ((h) => 'N/A');
    const getMoonPhaseName = window.getMoonPhaseName || ((p) => 'N/A');
    const getSolarData = window.getSolarData || ((d, lat, lon, alt) => null);

    // =========================================================
    // BLOC 0 : GESTION DU TEMPS (syncH)
    // =========================================================

    function syncH() {
        const now = performance.now();
        const deltaTime = (now - lastTime) / 1000.0; 
        lastTime = now;
        
        if (!isGpsPaused) {
            currentSessionTime += deltaTime;
            if (currentUKFState.speed > 0.01) { currentMovementTime += deltaTime; } 
        }

        const localTime = new Date();
        
        // --- MISE À JOUR DU TEMPS ---
        if ($('heure-locale')) $('heure-locale').textContent = localTime.toTimeString().substring(0, 8) + ' (Local)';

        // 🟢 FIX CRITIQUE : FORCER L'AFFICHAGE DE L'HEURE UTC
        const utcDatePart = localTime.toLocaleDateString('fr-FR', {
            year: 'numeric', month: '2-digit', day: '2-digit', timeZone: 'UTC'
        }).replace(/(\d{2})\/(\d{2})\/(\d{4})/, '$1-$2-$3'); // Format J-M-A
        
        const utcTimePart = localTime.toLocaleTimeString('fr-FR', {
            hour: '2-digit', minute: '2-digit', second: '2-digit', timeZone: 'UTC'
        });

        if ($('utc-datetime')) {
            $('utc-datetime').textContent = utcDatePart + ' ' + utcTimePart + ' UTC/GMT';
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
        const V_ms = isGpsPaused && !isIMUActive ? 0.0 : (currentUKFState.speed || 0.0); 
        const M = currentMass;           
        const speed_kmh = V_ms * 3.6; 
        
        // 2. CALCULS PHYSIQUES & RELATIVISTES 
        const v_ratio_c = V_ms / C; 
        const gamma = 1 / Math.sqrt(1 - v_ratio_c * v_ratio_c);
        const dynamic_pressure = 0.5 * RHO_AIR_ISA * V_ms * V_ms; 
        const kinetic_energy = 0.5 * M * V_ms * V_ms; 
        const mach_number = V_ms / V_SOUND_ISA; 
        
        // 3. CALCULS ASTRO (Utilise les fonctions de astro.js et ephem.js)
        const today = new Date();
        let astroData = null;
        try {
            // Utiliser la fonction globale (assure l'accès si astro.js est chargé)
            if (typeof window.getSolarData === 'function') {
                astroData = window.getSolarData(today, currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
            }
        } catch (e) {
            // Laisse astroData à null
        }
        
        // --- MISE À JOUR DOM : VITESSE & RELATIVITÉ ---
        
        // Vitesse (En mode PAUSE/ATTENTE, les tirets sont appropriés)
        const speedFallback = isGpsPaused ? '0,0 km/h' : '--.- km/h';
        const speedMSFallback = isGpsPaused ? '0,00 m/s' : '-- m/s';
        if ($('current-speed-kmh')) $('current-speed-kmh').textContent = dataOrDefault(speed_kmh, 1, ' km/h', speedFallback, false); 
        if ($('stable-speed-ms')) $('stable-speed-ms').textContent = dataOrDefault(V_ms, 2, ' m/s', speedMSFallback, false); 
        
        // Physique & Relativité
        if ($('perc-vitesse-son')) $('perc-vitesse-son').textContent = dataOrDefault(V_ms / V_SOUND_ISA * 100, 2, ' %'); 
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach_number, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(v_ratio_c * 100, 2, ' %'); 
        if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefault(gamma, 4);
        
        // --- MISE À JOUR DOM : DYNAMIQUE & EKF DEBUG ---
        
        // Dynamique & Forces
        // 🟢 FIX CRITIQUE: Appel à getGravity (qui DOIT être ajouté dans ukf-lib.js)
        const calculatedGravity = (typeof window.getGravity === 'function') 
            ? window.getGravity(currentUKFState.lat * D2R, currentUKFState.alt) 
            : G_STD; // Fallback à 9.8067 m/s² si la fonction manque
            
        if ($('local-gravity')) $('local-gravity').textContent = dataOrDefault(calculatedGravity, 4, ' m/s²'); 
        
        // Mécanique des Fluides & Champs
        if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(dynamic_pressure, 2, ' Pa');
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kinetic_energy, 2, ' J'); 
        
        // Filtre EKF/UKF & Debug
        const gpsStatusText = isGpsPaused ? 'INACTIF' : 'ATTENTE SIGNAL';
        if ($('gps-status')) $('gps-status').textContent = gpsStatusText; 
        if ($('ekf-status')) $('ekf-status').textContent = isGpsPaused ? 'INACTIF' : 'ACQUISITION';
        
        // --- MISE À JOUR DOM : POSITION & ASTRO ---
        
        // Position
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 

        // Astro
        if (astroData) {
            // TST/MST
            if ($('tst-time')) $('tst-time').textContent = formatHours(astroData.TST_HRS);
            if ($('mst-time')) $('mst-time').textContent = formatHours(astroData.MST_HRS);
            if ($('equation-of-time')) $('equation-of-time').textContent = dataOrDefault(astroData.EOT_MIN, 2, ' min'); 
            
            // Soleil
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.position.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.position.azimuth * R2D, 2, '°'); 
            
            // Lune
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(astroData.moon.illumination.phase);
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 2, '°');
            if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(astroData.moon.position.azimuth * R2D, 2, '°'); 
        } else {
             // Fallbacks for Astro
             if ($('tst-time')) $('tst-time').textContent = 'N/A';
             if ($('mst-time')) $('mst-time').textContent = 'N/A';
             if ($('equation-of-time')) $('equation-of-time').textContent = 'N/A';
             if ($('sun-alt')) $('sun-alt').textContent = 'N/A';
             if ($('sun-azimuth')) $('sun-azimuth').textContent = 'N/A'; 
             if ($('moon-phase-name')) $('moon-phase-name').textContent = 'N/A';
             if ($('moon-alt')) $('moon-alt').textContent = 'N/A';
             if ($('moon-azimuth')) $('moon-azimuth').textContent = 'N/A'; 
        }
    } // Fin de updateDashboard

    // =========================================================
    // BLOC 7 : INITIALISATION DU SYSTÈME
    // =========================================================

    window.addEventListener('load', () => {
        
        // Initialisation UKF (doit se faire après le chargement de math.js et ukf-lib.js)
        // Vérifie si la classe ProfessionalUKF est disponible
        if (typeof window.ProfessionalUKF === 'function') { 
            ukf = new ProfessionalUKF();
        }
        
        // 🟢 FIX CRITIQUE: Tenter d'initialiser les capteurs (pour mise à jour statut)
        if (typeof window.initIMUSensors === 'function') {
            window.initIMUSensors();
            isIMUActive = true;
        }
        if (typeof window.initEnvironmentalSensors === 'function') {
            window.initEnvironmentalSensors();
        }
        
        // MISE À JOUR INITIALE DU STATUT DES CAPTEURS
        if ($('imu-status') && !isIMUActive) {
            $('imu-status').textContent = 'Inactif (Non-démarré)';
        }

        // Exécution immédiate
        syncH(); 
        updateDashboard(); 
        
        // Exécution à haute fréquence (60Hz) pour garantir la mise à jour des valeurs.
        setInterval(() => {
            syncH();
            updateDashboard();
        }, 1000 / 60); 

    });

})(window);
