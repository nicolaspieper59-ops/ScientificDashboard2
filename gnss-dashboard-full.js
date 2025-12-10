// =================================================================
// FICHIER : gnss-dashboard-full.js (V7.3 - FINAL ET DÉFINITIF)
// VERSION : INCLUSION DE TOUS LES CORRECTIFS D'ID ET DE L'HEURE UTC
// DÉPENDANCES CRITIQUES: lib/ukf-lib.js (avec getGravity), lib/astro.js (avec getSolarData)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique avec une précision fixe, ou retourne la valeur par défaut.
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
 * Formate une valeur numérique en notation exponentielle.
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) < 1e-30) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals).replace('.', ',');
};


// --- CONSTANTES PHYSIQUES ET CONVERSIONS ---
const C = 299792458.0;              
const G_STD = 9.8067;               
const RHO_AIR_ISA = 1.225;          
const V_SOUND_ISA = 340.2900;       
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180; 

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // --- ÉTATS GLOBAUX INITIAUX ---
    let ukf = null; 
    let isGpsPaused = true; 
    let isIMUActive = false;            
    let currentMass = 70.0;             
    let currentMaxSpeed_ms = 0.0 / 3.6;    
    let currentSessionTime = 0.00;       
    let currentMovementTime = 0.00;
    
    // Coordonnées initiales (Marseille par défaut)
    let currentUKFState = { 
        lat: 43.284572, lon: 5.358710, alt: 100.00, 
        vN: 0.0, vE: 0.0, vD: 0.0, 
        speed: 0.0, kUncert: 0.0 
    };
    
    let lastTime = performance.now();
    
    // --- VÉRIFICATION ET FALLBACKS DES DÉPENDANCES ASTRO (de lib/astro.js) ---
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
        
        // --- MISE À JOUR DU TEMPS LOCAL ---
        if ($('heure-locale')) $('heure-locale').textContent = localTime.toTimeString().substring(0, 8) + ' (Local)';

        // 🟢 FIX CRITIQUE: AFFICHAGE STABLE DE L'HEURE UTC
        try {
            const utcDatePart = localTime.toLocaleDateString('fr-FR', {
                year: 'numeric', month: '2-digit', day: '2-digit', timeZone: 'UTC'
            }).replace(/\//g, '-');
            
            const utcTimePart = localTime.toLocaleTimeString('fr-FR', {
                hour: '2-digit', minute: '2-digit', second: '2-digit', timeZone: 'UTC'
            });

            if ($('utc-datetime')) {
                $('utc-datetime').textContent = `${utcDatePart} ${utcTimePart} UTC/GMT`;
            }
        } catch (e) {
            if ($('utc-datetime')) $('utc-datetime').textContent = 'N/A (Erreur)';
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
        
        // 2. CALCULS PHYSIQUES & RELATIVISTES 
        const v_ratio_c = V_ms / C; 
        const gamma = 1 / Math.sqrt(1 - v_ratio_c * v_ratio_c);
        const dynamic_pressure = 0.5 * RHO_AIR_ISA * V_ms * V_ms; 
        const kinetic_energy = 0.5 * currentMass * V_ms * V_ms; 
        
        // 3. CALCULS ASTRO 
        const today = new Date();
        let astroData = null;
        try {
            // Tente de calculer les données astro si la fonction est définie
            if (typeof window.getSolarData === 'function') {
                astroData = window.getSolarData(today, currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
            }
        } catch (e) {
            // Laisse astroData à null et affiche l'erreur dans la console si nécessaire
            console.error("Erreur critique lors du calcul Astro. Vérifiez astro.js et ses dépendances.", e);
        }
        
        // --- MISE À JOUR DOM : DYNAMIQUE & FORCES ---
        
        // Gravité Locale
        const calculatedGravity = (typeof window.getGravity === 'function') 
            ? window.getGravity(currentUKFState.lat * D2R, currentUKFState.alt) 
            : G_STD; 
            
        // ID: local-gravity
        if ($('local-gravity')) $('local-gravity').textContent = dataOrDefault(calculatedGravity, 4, ' m/s²'); 
        
        // Énergie Cinétique (ID unifié)
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kinetic_energy, 2, ' J'); 
        if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(dynamic_pressure, 2, ' Pa');


        // --- MISE À JOUR DOM : POSITION EKF (Fonctionnel) ---
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 

        // --- MISE À JOUR DOM : ASTRO ---
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
            // 🟢 FIX CRITIQUE: Utilisation de l'ID "moon-illuminated"
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 1, ' %');
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 2, '°');
            if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(astroData.moon.position.azimuth * R2D, 2, '°'); 
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefaultExp(astroData.moon.position.distance, 2, ' m');
        } else {
             // Fallbacks pour tous les champs Astro
             if ($('tst-time')) $('tst-time').textContent = 'N/A';
             if ($('mst-time')) $('mst-time').textContent = 'N/A';
             if ($('equation-of-time')) $('equation-of-time').textContent = 'N/A';
             if ($('sun-alt')) $('sun-alt').textContent = 'N/A';
             if ($('sun-azimuth')) $('sun-azimuth').textContent = 'N/A'; 
             if ($('moon-phase-name')) $('moon-phase-name').textContent = 'N/A';
             if ($('moon-illuminated')) $('moon-illuminated').textContent = 'N/A'; // ID Corrigé
             if ($('moon-alt')) $('moon-alt').textContent = 'N/A';
             if ($('moon-azimuth')) $('moon-azimuth').textContent = 'N/A'; 
             if ($('moon-distance')) $('moon-distance').textContent = 'N/A';
        }
    } // Fin de updateDashboard

// =========================================================
// BLOC 7 : INITIALISATION
// =========================================================

    window.addEventListener('load', () => {
        
        // Initialisation UKF 
        if (typeof window.ProfessionalUKF === 'function') { 
            ukf = new ProfessionalUKF();
        }

        // 1. Initialisation des fonctions de base
        syncH(); // Appel immédiat pour l'heure

        // 2. Mise à jour initiale des statuts (Gravity, Astro, Capteurs)
        updateDashboard();

        // 3. Boucle principale de rafraîchissement (60Hz)
        setInterval(() => {
            syncH(); 
            updateDashboard(); 
        }, 1000 / 60);
        
    });

})(window);
