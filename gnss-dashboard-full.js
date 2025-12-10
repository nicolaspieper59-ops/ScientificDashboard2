// =================================================================
// FICHIER : gnss-dashboard-full.js (V8.2 - DIAGNOSTIC FINAL)
// VERSION : SÉCURITÉ MAXIMALE - Test de tous les IDs critiques
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

// --- CONSTANTES PHYSIQUES ET CONVERSIONS ---
const C = 299792458.0;              
const G_STD = 9.8067;               
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180; 

// --- FORMATAGE ROBUSTE ---
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

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // --- ÉTATS GLOBAUX INITIAUX ---
    let ukf = null; 
    let isGpsPaused = true; 
    let currentSessionTime = 0.00;       
    let currentMovementTime = 0.00;
    
    // Coordonnées initiales (Marseille par défaut, testées comme fonctionnelles)
    let currentUKFState = { 
        lat: 43.284572, lon: 5.358710, alt: 100.00, 
        speed: 0.0, 
    };
    
    let lastTime = performance.now();
    
    // --- FALLBACKS ---
    // Si la librairie astro n'est pas chargée (problème le plus probable)
    const formatHours = window.formatHours || ((h) => 'N/A');
    const getMoonPhaseName = window.getMoonPhaseName || ((p) => 'N/A');
    // Si la librairie ukf-lib n'est pas chargée, nous utilisons la gravité standard G_STD
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
        
        // --- MISE À JOUR DU TEMPS LOCAL & UTC ---
        
        // ID: local-time-ntp -> Affichage avec (Local) pour le matching HTML
        if ($('local-time-ntp')) $('local-time-ntp').textContent = localTime.toTimeString().substring(0, 8) + ' (Local)';

        // 🟢 FIX CRITIQUE V8.2: Date & Heure (UTC/GMT) -> ID: utc-datetime
        try {
            // Utilisation d'un format simple pour éviter les erreurs de locale.
            const dateUTC = localTime.getUTCFullYear() + '-' + (localTime.getUTCMonth() + 1).toString().padStart(2, '0') + '-' + localTime.getUTCDate().toString().padStart(2, '0');
            const timeUTC = localTime.getUTCHours().toString().padStart(2, '0') + ':' + localTime.getUTCMinutes().toString().padStart(2, '0') + ':' + localTime.getUTCSeconds().toString().padStart(2, '0');

            if ($('utc-datetime')) {
                $('utc-datetime').textContent = `${dateUTC} ${timeUTC} UTC/GMT`;
            }
        } catch (e) {
            if ($('utc-datetime')) $('utc-datetime').textContent = 'N/A (Err Time)';
        }

        // Temps de session (Déjà OK)
        if ($('elapsed-session-time')) $('elapsed-session-time').textContent = dataOrDefault(currentSessionTime, 2, ' s'); 
        if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = dataOrDefault(currentMovementTime, 2, ' s');
    }

// =========================================================
// BLOC 1 : LOGIQUE DE CALCUL CRITIQUE (Gravité, Astro)
// =========================================================

    function updateDashboard() {
        
        // --- MISE À JOUR DOM : DYNAMIQUE & FORCES (GRAVITÉ) ---
        try {
            // Gravité Locale
            // 🟢 TEST CRITIQUE: Si l'ID est correct, ceci doit afficher 9.8067 ou une valeur calculée.
            const calculatedGravity = getGravity(currentUKFState.lat * D2R, currentUKFState.alt);
            if ($('local-gravity')) {
                 $('local-gravity').textContent = dataOrDefault(calculatedGravity, 4, ' m/s²'); 
            }
        } catch (e) {
             console.error("Erreur de calcul ou ID 'local-gravity' non trouvé.");
             if ($('local-gravity')) $('local-gravity').textContent = 'N/A (Grav Fail)';
        }


        // --- MISE À JOUR DOM : POSITION EKF (Déjà OK) ---
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 

        // --- MISE À JOUR DOM : ASTRO ---
        let astroData = null;
        
        try {
            // Tente de calculer les données astro SEULEMENT si l'API est définie globalement
            if (typeof window.getSolarData === 'function') {
                astroData = window.getSolarData(new Date(), currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
            }
        } catch (e) {
            console.error("Erreur critique d'exécution dans getSolarData (problème astro.js ou dépendances).", e);
            // astroData reste null si échec
        }
        
        // Bloc d'affichage Astro
        const astro_na = 'N/A';
        if (astroData) {
            // FIX CRITIQUES (TST, MST, EOT)
            if ($('tst')) $('tst').textContent = formatHours(astroData.TST_HRS);
            if ($('mst')) $('mst').textContent = formatHours(astroData.MST_HRS);
            if ($('eot')) $('eot').textContent = dataOrDefault(astroData.EOT_MIN, 2, ' min'); 
            
            // Soleil (exemple de mise à jour si Astro fonctionne)
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.position.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.position.azimuth * R2D, 2, '°'); 
            
            // Lune
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(astroData.moon.illumination.phase);
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 1, ' %');
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(astroData.moon.position.distance, 0, ' m', 'N/A', false);
            
            // Fallback pour les autres champs Astro non affichés par défaut
            if ($('date-display-astro')) $('date-display-astro').textContent = astroData.dateStr || astro_na;
        } else {
             // Fallbacks pour TOUS les champs Astro (doit garantir N/A si le calcul crash)
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
             if ($('date-display-astro')) $('date-display-astro').textContent = astro_na;
        }
    } // Fin de updateDashboard

// =========================================================
// BLOC 7 : INITIALISATION
// =========================================================

    window.addEventListener('load', () => {
        
        // Initialisation UKF (Tentative)
        if (typeof window.ProfessionalUKF === 'function') { 
            ukf = new ProfessionalUKF();
        }

        // Boucle principale de rafraîchissement (60Hz)
        setInterval(() => {
            syncH(); 
            updateDashboard(); 
        }, 1000 / 60);
        
        // Exécution immédiate
        syncH(); 
        updateDashboard(); 
    });

})(window);
