// =================================================================
// FICHIER : gnss-dashboard-full.js (V8.1 - ADAPTATION COMPLÈTE À TOUS LES ID HTML)
// VERSION : ULTRA-DÉFENSIVE - Gravité et Temps Astro synchronisés ou avec fallback
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

// --- CONSTANTES PHYSIQUES ET CONVERSIONS ---
const C = 299792458.0;              
const G_STD = 9.8067;               
const RHO_AIR_ISA = 1.225;          
const V_SOUND_ISA = 340.2900;       
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180; 

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
    
    // --- VÉRIFICATION ET FALLBACKS DES DÉPENDANCES ASTRO & GRAVITÉ ---
    // Ces fonctions DOIVENT être définies dans astro.js / ukf-lib.js
    const formatHours = window.formatHours || ((h) => 'N/A');
    const getMoonPhaseName = window.getMoonPhaseName || ((p) => 'N/A');
    // Fallback direct à la gravité standard (G_STD) si getGravity n'est pas défini
    const getGravity = window.getGravity || ((latRad, alt) => G_STD); 
    

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
        // ID: local-time-ntp -> Affichage simple de l'heure locale
        if ($('local-time-ntp')) $('local-time-ntp').textContent = localTime.toTimeString().substring(0, 8);

        // 🟢 FIX CRITIQUE: Date & Heure (UTC/GMT) -> ID: utc-datetime
        try {
            // Utilisation d'une méthode plus robuste pour l'affichage UTC
            const utcTime = localTime.toUTCString();
            // Nettoyage: Retire le jour de la semaine et ajoute "UTC/GMT"
            const cleanedUtcTime = utcTime.substring(utcTime.indexOf(',') + 2).replace("GMT", "UTC/GMT");
            if ($('utc-datetime')) {
                $('utc-datetime').textContent = cleanedUtcTime;
            }
        } catch (e) {
            if ($('utc-datetime')) $('utc-datetime').textContent = 'N/A (Err Time)';
        }


        // 🟢 ID CORRECT: Temps écoulé (Session) -> ID: elapsed-session-time
        if ($('elapsed-session-time')) $('elapsed-session-time').textContent = dataOrDefault(currentSessionTime, 2, ' s'); 
        // 🟢 ID CORRECT: Temps de Mouvement -> ID: elapsed-motion-time
        if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = dataOrDefault(currentMovementTime, 2, ' s');
    }

// =========================================================
// BLOC 1 : LOGIQUE DE CALCUL CRITIQUE (UKF/Physique/Astro)
// =========================================================

    function updateDashboard() {
        
        // 1. DÉFINITION DE L'ÉTAT ACTUEL (Vitesse pour les calculs)
        const V_ms = isGpsPaused && !isIMUActive ? 0.0 : (currentUKFState.speed || 0.0); 
        
        // 2. CALCULS PHYSIQUES & RELATIVISTES 
        const v_ratio_c = V_ms / C; 
        const dynamic_pressure = 0.5 * RHO_AIR_ISA * V_ms * V_ms; 
        const kinetic_energy = 0.5 * currentMass * V_ms * V_ms; 
        
        // 3. CALCULS ASTRO 
        const today = new Date();
        let astroData = null;
        let astro_is_functional = (typeof window.getSolarData === 'function');
        
        if (astro_is_functional) {
            try {
                // Tente d'obtenir les données astro
                astroData = window.getSolarData(today, currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
            } catch (e) {
                console.error("Erreur critique lors du calcul Astro. L'API est définie mais a échoué. Détails:", e);
                // Si l'API échoue, astroData reste null, ce qui active le bloc 'else' de fallback
            }
        }
        
        // --- MISE À JOUR DOM : DYNAMIQUE & FORCES ---
        
        // Gravité Locale
        // 🟢 FIX CRITIQUE: Gravité Locale (g) -> ID: local-gravity
        // Utilisation de la fonction getGravity avec le fallback standard intégré.
        const calculatedGravity = getGravity(currentUKFState.lat * D2R, currentUKFState.alt);
        if ($('local-gravity')) $('local-gravity').textContent = dataOrDefault(calculatedGravity, 4, ' m/s²'); 
        
        // Énergie/Pression (Déjà OK)
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kinetic_energy, 2, ' J'); 
        if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(dynamic_pressure, 2, ' Pa');


        // --- MISE À JOUR DOM : POSITION EKF (Déjà OK) ---
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 

        // --- MISE À JOUR DOM : ASTRO ---
        if (astroData) {
            // FIX CRITIQUES (TST, MST, EOT)
            if ($('tst')) $('tst').textContent = formatHours(astroData.TST_HRS);
            if ($('mst')) $('mst').textContent = formatHours(astroData.MST_HRS);
            if ($('eot')) $('eot').textContent = dataOrDefault(astroData.EOT_MIN, 2, ' min'); 
            
            // Astro (Soleil, Lune, etc. - Utilise les autres IDs HTML)
             if ($('date-display-astro')) $('date-display-astro').textContent = astroData.dateStr || 'N/A';
             if ($('date-solar-mean')) $('date-solar-mean').textContent = astroData.dateSolarMean || 'N/A';
             if ($('date-solar-true')) $('date-solar-true').textContent = astroData.dateSolarTrue || 'N/A';
             if ($('noon-solar')) $('noon-solar').textContent = astroData.NOON_SOLAR_UTC || 'N/A';
             if ($('tslv')) $('tslv').textContent = astroData.TSLV_HRS || 'N/A';
             if ($('ecl-long')) $('ecl-long').textContent = dataOrDefault(astroData.ECL_LONG * R2D, 2, '°');
             
            // Soleil
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.position.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.position.azimuth * R2D, 2, '°'); 
            // if ($('day-duration')) $('day-duration').textContent = dataOrDefault(astroData.dayDurationHrs, 2, ' h'); // Assurez-vous que dayDurationHrs existe dans astroData
            
            // Lune
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(astroData.moon.illumination.phase);
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 1, ' %');
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 2, '°');
            if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(astroData.moon.position.azimuth * R2D, 2, '°'); 
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefaultExp(astroData.moon.position.distance, 2, ' m');
        } else {
             // Fallbacks pour tous les champs Astro
             const astro_na = 'N/A';
             if ($('tst')) $('tst').textContent = astro_na;
             if ($('mst')) $('mst').textContent = astro_na;
             if ($('eot')) $('eot').textContent = astro_na;
             if ($('sun-alt')) $('sun-alt').textContent = astro_na;
             if ($('sun-azimuth')) $('sun-azimuth').textContent = astro_na; 
             if ($('moon-phase-name')) $('moon-phase-name').textContent = astro_na;
             if ($('moon-illuminated')) $('moon-illuminated').textContent = astro_na;
             if ($('moon-alt')) $('moon-alt').textContent = astro_na;
             if ($('moon-azimuth')) $('moon-azimuth').textContent = astro_na; 
             if ($('moon-distance')) $('moon-distance').textContent = astro_na;
             
             // Autres champs Astro
             if ($('date-display-astro')) $('date-display-astro').textContent = astro_na;
             if ($('date-solar-mean')) $('date-solar-mean').textContent = astro_na;
             if ($('date-solar-true')) $('date-solar-true').textContent = astro_na;
             if ($('noon-solar')) $('noon-solar').textContent = astro_na;
             if ($('tslv')) $('tslv').textContent = astro_na;
             if ($('ecl-long')) $('ecl-long').textContent = astro_na;
             if ($('day-duration')) $('day-duration').textContent = astro_na;
             if ($('sunrise-times')) $('sunrise-times').textContent = astro_na;
             if ($('sunset-times')) $('sunset-times').textContent = astro_na;
             if ($('moon-times')) $('moon-times').textContent = astro_na;
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
