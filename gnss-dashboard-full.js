// =================================================================
// FICHIER : gnss-dashboard-full.js (V8.3 - DIAGNOSTIC ULTIME)
// VERSION : SÉCURITÉ MAXIMALE & ISOLATION DES CRASH
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

// --- CONSTANTES PHYSIQUES ET CONVERSIONS ---
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
// DÉMARRAGE : Encapsulation de la logique (IIFE)
// =================================================================

((window) => {

    // --- ÉTATS GLOBAUX INITIAUX ---
    let ukf = null; 
    let isGpsPaused = true; 
    let currentSessionTime = 0.00;       
    let currentMovementTime = 0.00;
    
    let currentUKFState = { 
        lat: 43.284572, lon: 5.358710, alt: 100.00, 
        speed: 0.0, 
    };
    
    let lastTime = performance.now();
    
    // --- FALLBACKS ---
    const formatHours = window.formatHours || ((h) => 'N/A');
    const getMoonPhaseName = window.getMoonPhaseName || ((p) => 'N/A');
    const getGravity = window.getGravity || ((latRad, alt) => G_STD); 
    
    
// =========================================================
// BLOC 0 : GESTION DU TEMPS (syncH)
// =========================================================

    function syncH() {
        try {
            const now = performance.now();
            const deltaTime = (now - lastTime) / 1000.0; 
            lastTime = now;
            
            if (!isGpsPaused) {
                currentSessionTime += deltaTime;
                if (currentUKFState.speed > 0.01) { currentMovementTime += deltaTime; } 
            }

            const localTime = new Date();
            
            // 🟢 Heure Locale (NTP)
            if ($('local-time-ntp')) $('local-time-ntp').textContent = localTime.toTimeString().substring(0, 8) + ' (Local)';

            // 🟢 FIX CRITIQUE V8.3: Date & Heure (UTC/GMT) -> ID: utc-datetime
            // Utilisation d'une méthode JavaScript native ultra-simple pour garantir l'affichage
            if ($('utc-datetime')) {
                 const utcString = localTime.toISOString().replace('T', ' ').substring(0, 19);
                 $('utc-datetime').textContent = `${utcString} UTC/GMT`;
            }

            // 🟢 Temps écoulé (Session) et Mouvement
            if ($('elapsed-session-time')) $('elapsed-session-time').textContent = dataOrDefault(currentSessionTime, 2, ' s'); 
            if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = dataOrDefault(currentMovementTime, 2, ' s');
        
        } catch (e) {
            console.error("Erreur critique dans syncH. L'horloge est bloquée.", e);
            if ($('utc-datetime')) $('utc-datetime').textContent = 'N/A (CRASH H)';
        }
    }

// =========================================================
// BLOC 1 : LOGIQUE DE CALCUL CRITIQUE (Gravité, Astro)
// =========================================================

    function updateDashboard() {
        
        // --- MISE À JOUR DOM : DYNAMIQUE & FORCES (GRAVITÉ) ---
        try {
            // Gravité Locale (g)
            // 🟢 TEST CRITIQUE: Cette ligne doit garantir une valeur numérique ou un crash non masqué.
            const calculatedGravity = getGravity(currentUKFState.lat * D2R, currentUKFState.alt);
            if ($('local-gravity')) {
                 $('local-gravity').textContent = dataOrDefault(calculatedGravity, 4, ' m/s²'); 
            }
        } catch (e) {
             console.error("Erreur critique lors du calcul de la Gravité (getGravity). L'ID HTML est peut-être incorrect ou ukf-lib.js est cassé.", e);
             if ($('local-gravity')) $('local-gravity').textContent = 'N/A (CRASH G)';
        }

        // --- MISE À JOUR DOM : ASTRO ---
        let astroData = null;
        
        try {
            // Tente de calculer les données astro SEULEMENT si l'API est définie globalement
            if (typeof window.getSolarData === 'function') {
                astroData = window.getSolarData(new Date(), currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
            }
        } catch (e) {
            console.error("Erreur critique d'exécution dans getSolarData. astro.js ou ses dépendances sont corrompus.", e);
        }
        
        // Bloc d'affichage Astro
        const astro_na = 'N/A';
        if (astroData) {
            // 🟢 TST, MST, EOT
            if ($('tst')) $('tst').textContent = formatHours(astroData.TST_HRS);
            if ($('mst')) $('mst').textContent = formatHours(astroData.MST_HRS);
            if ($('eot')) $('eot').textContent = dataOrDefault(astroData.EOT_MIN, 2, ' min'); 
            
            // ... autres champs Astro ...
             if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.position.altitude * R2D, 2, '°');
             if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(astroData.moon.illumination.phase);
             // ...
        } else {
             // Affichage N/A si le calcul a échoué
             if ($('tst')) $('tst').textContent = astro_na;
             if ($('mst')) $('mst').textContent = astro_na;
             if ($('eot')) $('eot').textContent = astro_na;
             // ...
        }
    } // Fin de updateDashboard

// =========================================================
// BLOC 7 : INITIALISATION
// =========================================================

    window.addEventListener('load', () => {
        
        // ⚠️ VÉRIFICATION CRITIQUE D'INITIALISATION
        // Ceci est une ligne de diagnostic pour vérifier l'ID Gravité
        if ($('local-gravity')) {
             $('local-gravity').textContent = '9,8067 DIAGNOSTIC';
        }

        // Initialisation UKF 
        if (typeof window.ProfessionalUKF === 'function') { 
            ukf = new ProfessionalUKF();
        }

        // Boucle principale de rafraîchissement
        setInterval(() => {
            syncH(); 
            updateDashboard(); 
        }, 1000 / 60);
        
        // Exécution immédiate
        syncH(); 
        updateDashboard(); 
    });

})(window); 
