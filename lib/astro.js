// =================================================================
// lib/astro.js : VERSION ROBUSTE ET COMPLÈTE
// Corrige les formules de Temps Solaire Vrai (TST) et l'accès à VSOP2013.
// =================================================================

// --- CONSTANTES ---
const R2D = 180 / Math.PI; 
const D2R = Math.PI / 180; 

// --- FONCTIONS UTILITAIRES (Nécessaires pour les calculs) ---
function getJulianDay(date) {
    // JD = Jour Julien. 2440587.5 est JD au 00:00:00 du 1er Janvier 1970
    return (date.getTime() / 86400000.0) + 2440587.5;
}

function getJulianCenturies(JD) {
    // T = Siècles Juliens depuis J2000 (JD 2451545.0)
    return (JD - 2451545.0) / 36525.0;
}

function getTSLV_hours(date, longitude, JD) {
    // Temps Sidéral de Greenwich (GST)
    const T = getJulianCenturies(JD);
    const Theta0 = 280.46061837 + 360.98564736629 * (JD - 2451545.0) + 0.000387933 * T**2 - T**3 / 38710000.0;
    
    // TSLV = TST de Greenwich + Longitude
    let TSLV_deg = (Theta0 % 360.0 + longitude) % 360.0;
    if (TSLV_deg < 0) TSLV_deg += 360.0;
    
    return TSLV_deg / 15.0; // En heures
}


/**
 * Tente d'accéder à l'API VSOP2013 (l'état du Barycentre Terre-Lune).
 * @param {number} JY2K - Temps en années juliens depuis J2000 (T * 100).
 * @returns {Array|null} Tableau [X, Y, Z, ...] ou null.
 */
function getVSOP2013State(JY2K) {
    // Correction critique: Ajout de +0.5 pour le half-step (spécifique à certaines implémentations VSOP)
    const JY2K_corrected = JY2K + 0.5; 
    
    if (typeof vsop2013 !== 'undefined' && vsop2013.emb && typeof vsop2013.emb.state === 'function') {
        return vsop2013.emb.state(JY2K_corrected);
    }
    
    if (typeof ephem !== 'undefined' && ephem.vsop2013 && ephem.vsop2013.emb && typeof ephem.vsop2013.emb.state === 'function') {
        return ephem.vsop2013.emb.state(JY2K_corrected);
    }

    console.warn("API VSOP2013 non trouvée. Le script continue avec l'approximation séculaire.");
    return null; 
}


/**
 * Calcule le Temps Solaire Vrai (TST) et l'Équation du Temps (EOT).
 * @param {Date} date - L'objet Date en UTC.
 * @param {number} longitude - Longitude en degrés.
 * @returns {object} Données solaires.
 */
function getSolarTime(date, longitude) {
    const JD = getJulianDay(date);
    const T = getJulianCenturies(JD); 
    const JY2K = T * 100; // Années juliens depuis J2000

    let X = 1.0; let Y = 0.0; 
    
    // Tente de récupérer les données VSOP pour une haute précision
    const state = getVSOP2013State(JY2K);
    
    if (state && state.length >= 3) {
        // Utilise la position X, Y du Barycentre Terre-Lune (point opposé au Soleil)
        X = state[0]; Y = state[1]; 
    } else {
        // --- FALLBACK (Approximation Séculaire Simple) ---
        const L_mean_rad = (280.46646 + 36000.76983 * T) * D2R; 
        X = Math.cos(L_mean_rad);
        Y = Math.sin(L_mean_rad);
    }
    
    // --- CONVERSION & CALCULS ASTRONOMIQUES ---
    
    // Longitude écliptique du Soleil
    const L_ecliptique_rad = Math.atan2(Y, X); 
    const L_ecliptique_deg = L_ecliptique_rad * R2D;
    
    // Obliquité de l'écliptique
    const epsilon_rad = (23.439291 - 0.0130042 * T) * D2R;
    
    // Conversion de la longitude écliptique en ascension droite (alpha_rad)
    const alpha_rad = Math.atan2(Math.cos(epsilon_rad) * Math.sin(L_ecliptique_rad), Math.cos(L_ecliptique_rad)); 
    let alpha_H = (alpha_rad * R2D) / 15.0; 
    if (alpha_H < 0) alpha_H += 24;

    // Temps Sidéral de Greenwich en heures (GST_H)
    const GST_H = getTSLV_hours(date, 0, JD); 
    
    // Équation du Temps (EOT = GSTV - Alpha)
    let EOT_H = (GST_H - alpha_H); // La conversion en 360.98... n'est pas nécessaire ici si on utilise GST
    EOT_H = (EOT_H + 36) % 24; // Permet de gérer les nombres négatifs
    if (EOT_H > 12) EOT_H -= 24; // Maintient EOT entre -12 et 12
    const EOT_minutes = EOT_H * 60.0;

    // --- CALCUL du TST (Temps Solaire Vrai) ---
    const UT_H = date.getUTCHours() + date.getUTCMinutes() / 60.0 + date.getUTCSeconds() / 3600.0;
    
    // TST = UTC + Longitude/15 + EOT
    // CORRECTION CRITIQUE (votre formule était incorrecte)
    let TST_hrs = UT_H + (longitude / 15.0) + EOT_H;

    TST_hrs = (TST_hrs % 24);
    if (TST_hrs < 0) TST_hrs += 24;

    const TST_H = Math.floor(TST_hrs).toString().padStart(2, '0');
    const TST_M = Math.floor((TST_hrs % 1) * 60).toString().padStart(2, '0');

    return {
        TST: `${TST_H}:${TST_M}`,
        EOT: EOT_minutes.toFixed(2),
        ECL_LONG: L_ecliptique_deg.toFixed(2)
    };
}


/**
 * Retourne le Temps Sidéral Local Vrai (TSLV) en heures (format H:M:S).
 */
function getTSLV(date, longitude) {
    const JD = getJulianDay(date);
    const TSLV_hours = getTSLV_hours(date, longitude, JD);
    
    const H = Math.floor(TSLV_hours).toString().padStart(2, '0');
    const M = Math.floor((TSLV_hours % 1) * 60);
    const S = Math.floor((TSLV_hours % 1 * 60) % 1 * 60);

    return `${H.padStart(2, '0')}:${M.toString().padStart(2, '0')}:${S.toString().padStart(2, '0')}`;
}


/**
 * Détermine le nom de la phase lunaire.
 * (Dépend souvent de SunCalc, mais garde l'utilitaire).
 */
function getMoonPhaseName(phase) {
    if (phase < 0.03) return "Nouvelle Lune";
    if (phase < 0.22) return "Premier Croissant";
    if (phase < 0.28) return "Premier Quartier";
    if (phase < 0.47) return "Lune Gibbeuse Croissante";
    if (phase < 0.53) return "Pleine Lune";
    if (phase < 0.72) return "Lune Gibbeuse Décroissante";
    if (phase < 0.78) return "Dernier Quartier";
    if (phase < 0.97) return "Dernier Croissant";
    return "Nouvelle Lune"; // Retour à la NL
}


// --- EXPOSITION GLOBALE (Nécessaire pour gnss-dashboard-full.js) ---
window.getSolarTime = getSolarTime;
window.getTSLV = getTSLV;
window.getMoonPhaseName = getMoonPhaseName;
// window.getMoonIllumination n'est pas définie ici, elle est normalement dans SunCalc
