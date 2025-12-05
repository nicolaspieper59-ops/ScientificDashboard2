// =================================================================
// lib/astro.js : Interface entre ephem.js et le Dashboard
// =================================================================

// --- CONSTANTES GLOBALES (nécessaires pour les conversions de temps) ---
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const T_J2000 = 2451545.0; // Jour Julien de référence (J2000.0)

// --- FONCTIONS UTILITAIRES DE BASE ---

/**
 * Calcule le Jour Julien (JJ) pour une date donnée (UTC).
 * @param {Date} date - L'objet Date en UTC.
 * @returns {number} Le jour Julien.
 */
function getJulianDay(date) {
    let year = date.getUTCFullYear();
    let month = date.getUTCMonth() + 1;
    let day = date.getUTCDate();
    const UT = (date.getUTCHours() + date.getUTCMinutes() / 60.0 + date.getUTCSeconds() / 3600.0) / 24.0;

    if (month <= 2) {
        year -= 1;
        month += 12;
    }

    const A = Math.floor(year / 100);
    const B = 2 - A + Math.floor(A / 4);

    let JD = Math.floor(365.25 * year) + Math.floor(30.6001 * (month + 1)) + day + 1720994.5;
    JD += UT;

    return JD;
}

/**
 * Calcule le temps en siècles juliens (T) ou années juliens (JY2K) depuis J2000.
 * @param {number} JD - Jour Julien.
 * @returns {number} Temps en siècles juliens (T).
 */
function getJulianCenturies(JD) {
    return (JD - T_J2000) / 36525.0; // T en siècles juliens
}


// --- FONCTIONS D'INTERFACE CRITIQUES (Appelées par gnss-dashboard-full.js) ---

/**
 * Fonction 1: getSolarTime(date, longitude)
 * Utilise ephem.js (vsop2013.emb) pour obtenir la position.
 * -----------------------------------------------------------------
 */
function getSolarTime(date, longitude) {
    const JD = getJulianDay(date);
    const T = getJulianCenturies(JD); 
    const JY2K = T * 100; // Temps en années juliens depuis J2000

    // ⚠️ UTILISATION DE LA LIBRAIRIE EPHEM.JS
    // vsop2013.emb.state(JY2K) retourne un tableau [X, Y, Z, VX, VY, VZ] du barycentre Terre-Lune
    let X = 0, Y = 0, Z = 0;
    
    try {
        // Le Soleil est calculé comme étant opposé au barycentre Terre-Lune (emb)
        // vsop2013.emb.state est l'API standard selon le README de THRASTRO/ephem.js
        const state = vsop2013.emb.state(JY2K); 
        X = state[0];
        Y = state[1];
        Z = state[2];
        
    } catch (e) {
        console.error("Erreur d'appel à vsop2013.emb.state. EPHEM.JS n'est pas chargé ou l'API a changé.");
        // Remplacement sécurisé pour éviter le plantage
        X = 1.0; Y = 0.0; Z = 0.0;
    }
    
    // --- CONVERSION DES COORDONNÉES RECTANGULAIRES (X, Y, Z) EN SPHÉRIQUES (L, B) ---
    // (Simplification : on suppose que le système de coordonnées de ephem.js est héliocentrique et écliptique J2000)
    
    const L_ecliptique_rad = Math.atan2(Y, X); // Longitude écliptique
    const R_distance = Math.sqrt(X * X + Y * Y + Z * Z); // Rayon vecteur (distance)
    
    // --- CALCUL DE L'ÉQUATION DU TEMPS (EOT) ET TST ---
    
    const GST_H = getTSLV_hours(date, 0, JD); // Temps Sidéral de Greenwich en heures (nécessaire pour EOT)
    const L_ecliptique_deg = L_ecliptique_rad * R2D;
    
    // Approximation de l'Équation du Temps (EOT, TST) par conversion L_ecliptique -> Alpha
    // Le calcul précis nécessite l'obliquité, mais on peut simplifier en supposant EOT est lié à la différence entre le mouvement réel (L) et moyen
    
    // Simplification pour l'Ascension Droite (Alpha) pour TST:
    const alpha_rad = Math.atan2(Math.sin(L_ecliptique_rad), Math.cos(L_ecliptique_rad)); 
    const alpha_H = (alpha_rad * R2D) / 15.0; // Alpha en heures
    
    // Temps Sidéral de Greenwich Vrai (GSTV) en heures
    const GSTV_H = GST_H * (360.0 / 360.985647366) // Conversion approximative Sidéral -> Solaire
    
    // Équation du Temps (EOT) en heures (GSTV - Alpha)
    const EOT_H = (GSTV_H - alpha_H) % 24; 
    const EOT_minutes = EOT_H * 60.0;

    // Heure Solaire Vraie (TST) en heures décimales (0-24)
    let TST_hrs = GSTV_H + (longitude / 15.0); // Approximation du TST local
    TST_hrs = TST_hrs % 24;
    if (TST_hrs < 0) TST_hrs += 24;

    // Conversion des heures décimales en HH:MM
    const TST_H = Math.floor(TST_hrs).toString().padStart(2, '0');
    const TST_M = Math.floor((TST_hrs % 1) * 60).toString().padStart(2, '0');

    return {
        MST: date.toTimeString().split(' ')[0],
        TST: `${TST_H}:${TST_M}`,
        EOT: EOT_minutes.toFixed(2), // EOT en minutes
        ECL_LONG: L_ecliptique_deg.toFixed(2)
    };
}


/**
 * Fonction 2: getTSLV(date, longitude)
 * Calcule le Temps Sidéral Local Vrai (TSLV).
 * -----------------------------------------------------------------
 */
function getTSLV(date, longitude) {
    const JD = getJulianDay(date);
    const TSL_H = getTSLV_hours(date, longitude, JD);
    
    const H = Math.floor(TSL_H).toString().padStart(2, '0');
    const M = Math.floor((TSL_H % 1) * 60).toString().padStart(2, '0');
    const S = Math.floor((((TSL_H % 1) * 60) % 1) * 60).toString().padStart(2, '0');

    return `${H}h ${M}m ${S}s`;
}

/**
 * Fonction interne pour le calcul du TSLV en heures.
 */
function getTSLV_hours(date, longitude, JD) {
    if (!JD) JD = getJulianDay(date);
    
    const T = getJulianCenturies(JD); 
    const UT_H = date.getUTCHours() + date.getUTCMinutes() / 60.0 + date.getUTCSeconds() / 3600.0;
    
    // Temps Sidéral de Greenwich à 0h UT (approximatif)
    let TSG_0_deg = 100.46061837 + 36000.770053608 * T + 0.000387933 * T**2;
    TSG_0_deg = TSG_0_deg % 360;
    if (TSG_0_deg < 0) TSG_0_deg += 360;

    // Temps Sidéral Local (TSL) en degrés
    let TSL_deg = TSG_0_deg + UT_H * 15.04106864 + longitude;
    TSL_deg = TSL_deg % 360;
    if (TSL_deg < 0) TSL_deg += 360;
    
    return TSL_deg / 15.0; // TSL en heures
}


/**
 * Fonction 3: getMoonPhaseName(phase)
 * -----------------------------------------------------------------
 */
function getMoonPhaseName(phase) {
    if (phase === undefined || phase === null || isNaN(phase)) return "N/A";

    if (phase < 0.01 || phase >= 0.99) return "Nouvelle Lune";
    if (phase < 0.24) return "Premier Croissant";
    if (phase < 0.26) return "Premier Quartier";
    if (phase < 0.49) return "Gibbeuse Croissante";
    if (phase < 0.51) return "Pleine Lune";
    if (phase < 0.74) return "Gibbeuse Décroissante";
    if (phase < 0.76) return "Dernier Quartier";
    return "Dernier Croissant";
}
