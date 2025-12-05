// =================================================================
// lib/astro.js (VERSION DIAGNOSTIQUE)
// Doit définir les fonctions globales pour gnss-dashboard-full.js
// =================================================================

/**
 * Définit le Temps Solaire Moyen (MST) et le Temps Solaire Vrai (TST).
 * Cette fonction est l'une de celles qui utilise les données de vsop2013.js.
 */
function getSolarTime(date, longitude) {
    const hours = date.getUTCHours();
    const minutes = date.getUTCMinutes();
    
    // Le code complet utilise vsop2013.js. Ici, c'est une substitution simple.
    const tstValue = (hours + (minutes / 60) + (longitude / 15)) % 24;
    const tstHours = Math.floor(tstValue).toString().padStart(2, '0');
    const tstMinutes = Math.floor((tstValue - tstHours) * 60).toString().padStart(2, '0');

    return {
        MST: date.toTimeString().split(' ')[0], // Heure locale (simple)
        TST: `${tstHours}:${tstMinutes}`, // Heure Solaire Vraie approximative
        EOT: (Math.random() * 10 - 5).toFixed(2), // Équation du temps simulée
        ECL_LONG: (Math.random() * 360).toFixed(2) // Longitude Écliptique simulée
    };
}

/**
 * Définit le Temps Sidéral Local Vrai.
 */
function getTSLV(date, longitude) {
    // Placeholder pour le Temps Sidéral
    return date.toTimeString().split(' ')[0] + ' (Simulé)';
}

/**
 * Donne le nom de la phase de la Lune (utilise SunCalc).
 */
function getMoonPhaseName(phase) {
    if (phase === undefined || phase === null) return "N/A";
    if (phase < 0.01 || phase >= 0.99) return "Nouvelle Lune";
    if (phase < 0.24) return "Premier Croissant";
    if (phase < 0.26) return "Premier Quartier";
    if (phase < 0.49) return "Gibbeuse Croissante";
    if (phase < 0.51) return "Pleine Lune";
    if (phase < 0.74) return "Gibbeuse Décroissante";
    if (phase < 0.76) return "Dernier Quartier";
    return "Dernier Croissant";
}

// Assurez-vous que toutes les fonctions ci-dessus sont définies sans 'const' ou 'let' 
// pour qu'elles soient globales (ce qui est fait ici avec 'function').
