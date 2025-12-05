// =================================================================
// lib/astro.js : Pont entre les données VSOP2013, SunCalc et le Dashboard
// Nécessite : vsop2013.js (pour les séries planétaires)
// Définitions de fonctions globales pour le gnss-dashboard-full.js
// =================================================================

// --- CONSTANTES ---
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const T_J2000 = 2451545.0; // Jour Julien de référence (J2000.0)

// --- FONCTIONS ASTRONOMIQUES BASÉES SUR VSOP2013 ---

/**
 * Calcule le Jour Julien (JJ) pour une date donnée (UTC).
 * @param {Date} date - L'objet Date en UTC.
 * @returns {number} Le jour Julien.
 */
function getJulianDay(date) {
    // Calcul précis du jour Julien pour une date UTC
    const year = date.getUTCFullYear();
    const month = date.getUTCMonth() + 1;
    const day = date.getUTCDate();
    const hour = date.getUTCHours();
    const minute = date.getUTCMinutes();
    const second = date.getUTCSeconds();

    let A, B, C, D;
    
    // Années avant 0
    let Y = year;
    let M = month;

    if (M <= 2) {
        Y -= 1;
        M += 12;
    }

    A = Math.floor(Y / 100);
    B = 2 - A + Math.floor(A / 4);

    // Partie entière du jour (date sans l'heure)
    C = Math.floor(365.25 * Y);
    D = Math.floor(30.6001 * (M + 1));

    let JD = B + C + D + day + 1720994.5;
    
    // Ajout de la fraction de jour
    JD += (hour + minute / 60.0 + second / 3600.0) / 24.0;

    return JD;
}

/**
 * Calcule la longitude moyenne du Soleil, la longitude du périgée et l'obliquité de l'écliptique.
 * (Simplifié pour ce contexte, le code complet utiliserait des séries VSOP).
 * @param {number} JD - Jour Julien.
 * @returns {object} { L_m, omega, epsilon }
 */
function getFundamentalAngles(JD) {
    const T = (JD - T_J2000) / 36525.0; // Siècles juliens depuis J2000.0

    // Simplifications des formules de l'Obliquité (epsilon), Longitude Solaire Moyenne (L_m) et Longitude du Périgée (omega).
    const L_m = (280.46646 + 36000.76983 * T + 0.0003032 * T**2) % 360; // Longitude Solaire Moyenne (degré)
    const omega = 282.9404 + 4.70935e-5 * T; // Longitude du Périgée (degré)

    // Obliquité de l'écliptique (degré)
    const epsilon = 23.439291 - 0.0130042 * T - 0.00000016 * T**2 + 0.000000504 * T**3;

    return { 
        L_m: L_m, 
        omega: omega % 360, 
        epsilon: epsilon 
    };
}


/**
 * Fonction 1: getSolarTime(date, longitude)
 * Calcule l'Équation du Temps (EOT), le Temps Solaire Vrai (TST) et autres données.
 * Dépend des fonctions VSOP2013 (Sun_L, Sun_B, Sun_R) qui doivent être disponibles globalement.
 * -----------------------------------------------------------------
 */
function getSolarTime(date, longitude) {
    const JD = getJulianDay(date);
    const T = (JD - T_J2000) / 36525.0; // Siècles juliens depuis J2000.0

    let L_ecliptique = 0.0; 
    let B_ecliptique = 0.0;

    // ⚠️ CRITIQUE : Utilisation des fonctions VSOP2013
    // Les fonctions VSOP2013 (Sun_L, Sun_B, Sun_R) sont supposées être dans le scope global
    // si vsop2013.js est chargé correctement.
    try {
        if (typeof Sun_L === 'function') L_ecliptique = Sun_L(T); // Longitude du Soleil (rad)
        if (typeof Sun_B === 'function') B_ecliptique = Sun_B(T); // Latitude du Soleil (rad)
    } catch (e) {
        console.warn("Erreur d'exécution VSOP2013. Calculs astro approximatifs.", e);
        // Si VSOP2013 plante, on utilise les angles fondamentaux (moins précis)
        L_ecliptique = getFundamentalAngles(JD).L_m * D2R; 
        B_ecliptique = 0; 
    }

    const { epsilon } = getFundamentalAngles(JD); 
    const epsilon_rad = epsilon * D2R;

    // --- CONVERSION EN COORDONNÉES ÉQUATORIALES (AD/D) ---
    // Conversion de la Longitude et Latitude Écliptique (L, B) en Ascension Droite (alpha) et Déclinaison (delta)
    const cosL = Math.cos(L_ecliptique);
    const sinL = Math.sin(L_ecliptique);
    const cosE = Math.cos(epsilon_rad);
    const sinE = Math.sin(epsilon_rad);
    
    // Ascension Droite (alpha)
    let alpha_rad = Math.atan2(sinL * cosE - Math.tan(B_ecliptique) * sinE, cosL);
    if (alpha_rad < 0) alpha_rad += 2 * Math.PI;
    
    // Déclinaison (delta)
    const delta_rad = Math.asin(Math.sin(B_ecliptique) * cosE + Math.cos(B_ecliptique) * sinE * sinL);

    // --- CALCUL DE L'ÉQUATION DU TEMPS (EOT) ---
    const alpha_deg = alpha_rad * R2D;
    
    // Temps Solaire Moyen (MST) en heures décimales (0-24)
    const UT = date.getUTCHours() + date.getUTCMinutes() / 60.0 + date.getUTCSeconds() / 3600.0; 
    const GST_m_deg = getTSLV(date, 0, JD) * 15; // GST moyen en degrés (Approximation)

    // Équation du Temps (EOT) en degrés (GST - (alpha + L_m))
    const EOT_deg = GST_m_deg - alpha_deg; // Approximation
    const EOT_minutes = EOT_deg * 4; // 1 degré = 4 minutes (temps)
    
    // Heure Solaire Vraie (TST) en heures décimales (0-24)
    let TST_hrs = UT + (EOT_minutes / 60.0) + (longitude / 15.0);
    TST_hrs = TST_hrs % 24;
    if (TST_hrs < 0) TST_hrs += 24;

    // Conversion des heures décimales en HH:MM
    const TST_H = Math.floor(TST_hrs).toString().padStart(2, '0');
    const TST_M = Math.floor((TST_hrs % 1) * 60).toString().padStart(2, '0');
    
    const UT_H = date.getUTCHours().toString().padStart(2, '0');
    const UT_M = date.getUTCMinutes().toString().padStart(2, '0');


    return {
        DateMST: date, 
        DateTST: date, 
        MST: `${UT_H}:${UT_M} UTC`,
        TST: `${TST_H}:${TST_M}`,
        EOT: EOT_minutes.toFixed(2), // EOT en minutes
        ECL_LONG: (L_ecliptique * R2D).toFixed(2) // Longitude Écliptique en degrés
    };
}


/**
 * Fonction 2: getTSLV(date, longitude, JD)
 * Calcule le Temps Sidéral Local Vrai (TSLV).
 * @param {Date} date - L'objet Date en UTC.
 * @param {number} longitude - Longitude en degrés (pour le TSLV).
 * @param {number} [JD] - Jour Julien précalculé.
 * -----------------------------------------------------------------
 */
function getTSLV(date, longitude, JD) {
    if (!JD) JD = getJulianDay(date);
    
    const T = (JD - T_J2000) / 36525.0; 
    const UT_H = date.getUTCHours() + date.getUTCMinutes() / 60.0 + date.getUTCSeconds() / 3600.0;
    
    // Temps Sidéral de Greenwich à 0h UT (approximatif)
    let TSG_0_deg = 100.46061837 + 36000.770053608 * T + 0.000387933 * T**2 - 2.583e-8 * T**3;
    TSG_0_deg = TSG_0_deg % 360;
    if (TSG_0_deg < 0) TSG_0_deg += 360;

    // Temps Sidéral Local (TSL) en degrés
    let TSL_deg = TSG_0_deg + UT_H * 15.04106864 + longitude;
    TSL_deg = TSL_deg % 360;
    if (TSL_deg < 0) TSL_deg += 360;
    
    // Conversion en temps (heures)
    const TSL_H = TSL_deg / 15.0;

    const H = Math.floor(TSL_H).toString().padStart(2, '0');
    const M = Math.floor((TSL_H % 1) * 60).toString().padStart(2, '0');
    const S = Math.floor((((TSL_H % 1) * 60) % 1) * 60).toString().padStart(2, '0');

    return `${H}h ${M}m ${S}s`;
}


/**
 * Fonction 3: getMoonPhaseName(phase)
 * Donne le nom de la phase de la Lune à partir du fraction (SunCalc).
 * @param {number} phase - Fraction (0 à 1) d'illumination.
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
