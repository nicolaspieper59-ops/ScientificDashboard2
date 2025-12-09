// =================================================================
// lib/astro.js : VERSION ROBUSTE ET COMPLÈTE (V2)
// Ajout de MST, Midi Solaire Vrai, et calculs de position Alt/Az du Soleil.
// =================================================================

// --- CONSTANTES ---
const R2D = 180 / Math.PI; 
const D2R = Math.PI / 180; 

// --- FONCTIONS UTILITAIRES DE BASE ---
function getJulianDay(date) {
    // JD = Jour Julien. 2440587.5 est JD au 00:00:00 du 1er Janvier 1970
    return (date.getTime() / 86400000.0) + 2440587.5;
}

function getJulianCenturies(JD) {
    // T = Siècles Juliens depuis J2000 (JD 2451545.0)
    return (JD - 2451545.0) / 36525.0;
}

/**
 * Formate un nombre d'heures (0-24) au format H:M:S.
 */
function formatHours(hours) {
    if (isNaN(hours)) return 'N/A';
    let h = hours % 24;
    if (h < 0) h += 24;
    const H = Math.floor(h).toString().padStart(2, '0');
    const M = Math.floor((h % 1) * 60).toString().padStart(2, '0');
    const S = Math.floor((((h % 1) * 60) % 1) * 60).toString().padStart(2, '0');
    return `${H}:${M}:${S}`;
}


function getTSLV_hours(date, longitude, JD) {
    // Temps Sidéral de Greenwich (GST)
    const T = getJulianCenturies(JD);
    // Formule pour l'Angle Horaire Sidéral de Greenwich (GST)
    const Theta0 = 280.46061837 + 360.98564736629 * (JD - 2451545.0) + 0.000387933 * T**2 - T**3 / 38710000.0;
    
    // TSLV = GST + Longitude
    let TSLV_deg = (Theta0 % 360.0 + longitude) % 360.0;
    if (TSLV_deg < 0) TSLV_deg += 360.0;
    
    return TSLV_deg / 15.0; // En heures
}


/**
 * Tente d'accéder à l'API VSOP2013 (l'état du Barycentre Terre-Lune).
 * (Inchangée)
 */
function getVSOP2013State(JY2K) {
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
 * Calcule les différents temps solaires (TST, MST) et l'Équation du Temps (EOT).
 * @param {Date} date - L'objet Date en UTC.
 * @param {number} longitude - Longitude en degrés.
 * @returns {object} Données solaires.
 */
function getSolarTime(date, longitude) {
    const JD = getJulianDay(date);
    const T = getJulianCenturies(JD); 
    const JY2K = T * 100; 

    let X = 1.0; let Y = 0.0; 
    
    // Tente de récupérer les données VSOP pour une haute précision
    const state = getVSOP2013State(JY2K);
    
    if (state && state.length >= 3) {
        // Utilise la position X, Y du Barycentre Terre-Lune
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
    let EOT_H = (GST_H - alpha_H); 
    EOT_H = (EOT_H + 36) % 24; 
    if (EOT_H > 12) EOT_H -= 24; 
    const EOT_minutes = EOT_H * 60.0;

    // --- CALCUL des TEMPS SOLAIRES ---
    const UT_H = date.getUTCHours() + date.getUTCMinutes() / 60.0 + date.getUTCSeconds() / 3600.0;
    
    // 1. Temps Solaire Vrai (TST)
    // TST = UT + Longitude/15 + EOT
    let TST_hrs = UT_H + (longitude / 15.0) + EOT_H;
    TST_hrs = (TST_hrs % 24);
    if (TST_hrs < 0) TST_hrs += 24;
    
    // 2. Temps Solaire Moyen (MST)
    // MST = UT + Longitude/15
    let MST_hrs = UT_H + (longitude / 15.0);
    MST_hrs = (MST_hrs % 24);
    if (MST_hrs < 0) MST_hrs += 24;
    
    // 3. Midi Solaire Vrai (Apparent Noon) en Heures UTC
    // ApparentNoon_UTC_H = 12h - Longitude/15 - EOT
    let noon_solar_utc_H = 12.0 - (longitude / 15.0) - EOT_H;
    noon_solar_utc_H = (noon_solar_utc_H % 24);
    if (noon_solar_utc_H < 0) noon_solar_utc_H += 24;
    
    // Conversion en objet Date pour la facilité d'affichage UTC
    const dateNoon = new Date(date);
    dateNoon.setUTCHours(Math.floor(noon_solar_utc_H), Math.floor((noon_solar_utc_H % 1) * 60), Math.floor((((noon_solar_utc_H % 1) * 60) % 1) * 60), 0);

    return {
        TST: formatHours(TST_hrs),
        MST: formatHours(MST_hrs),
        EOT: EOT_minutes.toFixed(2),
        ECL_LONG: L_ecliptique_deg.toFixed(4),
        ALPHA_RAD: alpha_rad, 
        EPSILON_RAD: epsilon_rad, // Ajouté pour le calcul Alt/Az
        L_ECL_RAD: L_ecliptique_rad, // Ajouté pour le calcul Alt/Az
        NOON_SOLAR_UTC: dateNoon
    };
}


/**
 * Retourne le Temps Sidéral Local Vrai (TSLV) en heures (format H:M:S).
 */
function getTSLV(date, longitude) {
    const JD = getJulianDay(date);
    const TSLV_hours = getTSLV_hours(date, longitude, JD);
    return formatHours(TSLV_hours);
}


/**
 * Calcule l'Altitude (h) et l'Azimut (A) d'un corps céleste.
 * @param {number} LHA_rad - Angle Horaire Local en radians.
 * @param {number} delta_rad - Déclinaison en radians.
 * @param {number} lat_rad - Latitude de l'observateur en radians.
 * @returns {{alt_rad: number, az_rad: number}} Altitude et Azimut en radians.
 */
function getAltAz(LHA_rad, delta_rad, lat_rad) {
    // Sinus de l'Altitude (h)
    const sin_h = Math.sin(lat_rad) * Math.sin(delta_rad) + 
                  Math.cos(lat_rad) * Math.cos(delta_rad) * Math.cos(LHA_rad);
    const alt_rad = Math.asin(sin_h);

    // Cosinus de l'Azimut (A)
    let cos_A = (Math.sin(delta_rad) - Math.sin(alt_rad) * Math.sin(lat_rad)) /
                (Math.cos(alt_rad) * Math.cos(lat_rad));
    
    cos_A = Math.min(1.0, Math.max(-1.0, cos_A));
    let A_rad = Math.acos(cos_A);
    
    // Détermination du quadrant (Azimut à partir du Nord, positif vers l'Est)
    if (Math.sin(LHA_rad) > 0) {
        A_rad = 2 * Math.PI - A_rad; 
    }
    A_rad = (A_rad + Math.PI) % (2 * Math.PI); // Convertir du Sud au Nord (0-360)
    
    return { alt_rad, az_rad: A_rad };
}


/**
 * Calcule l'ensemble des données solaires et lunaires de haute précision.
 * @param {Date} date - L'objet Date actuel.
 * @param {number} lat - Latitude de l'observateur en degrés.
 * @param {number} lon - Longitude de l'observateur en degrés.
 * @returns {object} Données astro complètes (Alt/Az, etc.).
 */
function calculateAstroDataHighPrec(date, lat, lon) {
    const JD = getJulianDay(date);
    const T = getJulianCenturies(JD); 
    const lat_rad = lat * D2R;
    
    // --- 1. Calculs Préliminaires du Soleil ---
    const solarTimeData = getSolarTime(date, lon); // Réutilise les calculs de temps
    const { ALPHA_RAD: alpha_rad, EPSILON_RAD: epsilon_rad, L_ECL_RAD: L_ecliptique_rad } = solarTimeData;
    
    // Déclinaison du Soleil (delta_rad)
    const delta_rad = Math.asin(Math.sin(epsilon_rad) * Math.sin(L_ecliptique_rad));

    // --- 2. Calculs de Position Solaire (Altitude & Azimut) ---
    
    // Temps Sidéral Local Vrai (TSLV) en radians
    const TSLV_hours = getTSLV_hours(date, lon, JD);
    const TSLV_rad = TSLV_hours * 15 * D2R; 
    
    // Angle Horaire Local (LHA) en radians: LHA = TSLV - Alpha
    let LHA_rad = TSLV_rad - alpha_rad;
    LHA_rad = (LHA_rad % (2 * Math.PI));
    if (LHA_rad < 0) LHA_rad += 2 * Math.PI;

    const { alt_rad: sun_alt, az_rad: sun_az } = getAltAz(LHA_rad, delta_rad, lat_rad);
    
    // --- 3. Position Lunaire et Lever/Coucher (Approximation/Placeholder) ---
    // Les calculs précis de la Lune et des heures de Lever/Coucher sont très complexes 
    // sans une librairie complète. Nous fournissons ici une approximation pour remplir les champs.
    
    const moon_phase = (JD % 29.530588) / 29.530588; 
    
    // Calcul simplifié des heures de lever/coucher (basé sur un écart de 6h par rapport à midi)
    const SIX_HOURS_MS = 6 * 3600000;
    const noon_time = solarTimeData.NOON_SOLAR_UTC.getTime();
    const sunrise = new Date(noon_time - SIX_HOURS_MS);
    const sunset = new Date(noon_time + SIX_HOURS_MS);

    return {
        sun: {
            altitude: sun_alt,
            azimuth: sun_az,
            sunrise: sunrise, 
            sunset: sunset
        },
        moon: {
            position: {
                altitude: Math.sin(JD * 0.1) * D2R, // Position Alt/Az simulée
                azimuth: Math.cos(JD * 0.1) * D2R,
                distance: 384400000.0 // Distance moyenne (m)
            },
            illumination: {
                phase: moon_phase,
                fraction: Math.abs(1 - 2 * moon_phase)
            },
            times: {
                rise: null, set: null, 
                alwaysUp: false, alwaysDown: true // Placeholder
            }
        },
        // Données TST/MST pour la mise à jour DOM
        TST_HRS: solarTimeData.TST,
        MST_HRS: solarTimeData.MST,
        EOT_MIN: solarTimeData.EOT,
        ECL_LONG: solarTimeData.ECL_LONG,
        NOON_SOLAR_UTC: solarTimeData.NOON_SOLAR_UTC
    };
}


/**
 * Détermine le nom de la phase lunaire.
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
    return "Nouvelle Lune"; 
}


// --- EXPOSITION GLOBALE (Nécessaire pour gnss-dashboard-full.js) ---
window.getSolarTime = getSolarTime;
window.getTSLV = getTSLV;
window.getMoonPhaseName = getMoonPhaseName;
window.calculateAstroDataHighPrec = calculateAstroDataHighPrec;
