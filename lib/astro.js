// =================================================================
// lib/astro.js : FONCTION getSolarTime (CORRIGÉE pour Problème B)
// =================================================================

/**
 * Tente d'accéder à l'API VSOP2013, gérant les différents noms de librairie.
 * @param {number} JY2K - Temps en années juliens depuis J2000.
 * @returns {Array|null} Un tableau [X, Y, Z, ...] ou null en cas d'échec.
 */
function getVSOP2013State(JY2K) {
    // 1. Tente l'accès direct (comme dans THRASTRO/ephem.js)
    if (typeof vsop2013 !== 'undefined' && vsop2013.emb && typeof vsop2013.emb.state === 'function') {
        return vsop2013.emb.state(JY2K);
    }
    
    // 2. Tente l'accès via un objet global 'ephem' (fréquent)
    if (typeof ephem !== 'undefined' && ephem.vsop2013 && ephem.vsop2013.emb && typeof ephem.vsop2013.emb.state === 'function') {
        return ephem.vsop2013.emb.state(JY2K);
    }

    // 3. Tente l'accès si la librairie est encapsulée autrement
    if (typeof window.ephem_vsop2013_emb_state === 'function') {
        return window.ephem_vsop2013_emb_state(JY2K);
    }

    // Console.log pour le débogage si aucun chemin n'est trouvé
    console.warn("API VSOP2013 non trouvée. Utilisation de l'approximation séculaire.");
    return null; // Retourne null si la librairie n'est pas chargée
}


/**
 * Calcule le Temps Solaire et l'Équation du Temps (EOT).
 * @param {Date} date - L'objet Date en UTC.
 * @param {number} longitude - Longitude en degrés.
 * @returns {object} Données solaires.
 */
function getSolarTime(date, longitude) {
    const JD = getJulianDay(date);
    const T = getJulianCenturies(JD); 
    const JY2K = T * 100;

    let X = 1.0; 
    let Y = 0.0; 
    let Z = 0.0;
    
    // Tente de récupérer les données VSOP
    const state = getVSOP2013State(JY2K);
    
    if (state && state.length >= 3) {
        // Utilise la position [X, Y, Z] du Barycentre Terre-Lune (opposé au Soleil)
        X = state[0];
        Y = state[1];
        Z = state[2];
    } else {
        // --- FALLBACK (Approximation Séculaire Simple) ---
        // Utilise une formule classique si VSOP échoue
        const L_mean_rad = (280.46646 + 36000.76983 * T) * D2R; 
        X = Math.cos(L_mean_rad);
        Y = Math.sin(L_mean_rad);
        // Z reste 0 pour l'approximation
    }
    
    // --- CONVERSION & CALCUL (identique au code précédent) ---
    
    const L_ecliptique_rad = Math.atan2(Y, X); 
    
    const GST_H = getTSLV_hours(date, 0, JD); 
    const L_ecliptique_deg = L_ecliptique_rad * R2D;
    
    const alpha_rad = Math.atan2(Math.sin(L_ecliptique_rad), Math.cos(L_ecliptique_rad)); 
    const alpha_H = (alpha_rad * R2D) / 15.0; 
    
    const GSTV_H = GST_H * (360.0 / 360.985647366);
    
    let EOT_H = (GSTV_H - alpha_H) % 24; 
    if (EOT_H > 12) EOT_H -= 24; // Maintient EOT entre -12 et 12
    const EOT_minutes = EOT_H * 60.0;

    let TST_hrs = (date.getUTCHours() + date.getUTCMinutes() / 60.0 + date.getUTCSeconds() / 3600.0) + (longitude / 15.0) + (EOT_H - (date.getUTCHours() + date.getUTCMinutes() / 60.0 + date.getUTCSeconds() / 3600.0));
    TST_hrs = (TST_hrs) % 24;
    if (TST_hrs < 0) TST_hrs += 24;

    const TST_H = Math.floor(TST_hrs).toString().padStart(2, '0');
    const TST_M = Math.floor((TST_hrs % 1) * 60).toString().padStart(2, '0');

    return {
        MST: date.toTimeString().split(' ')[0], // Temps Solaire Moyen (approximatif)
        TST: `${TST_H}:${TST_M}`,
        EOT: EOT_minutes.toFixed(2),
        ECL_LONG: L_ecliptique_deg.toFixed(2)
    };
}
// Le reste de lib/astro.js (getTSLV, getMoonPhaseName, etc.) reste inchangé.
