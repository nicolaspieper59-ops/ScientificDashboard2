/**
 * lib/astro.js
 * Fonctions d'astrométrie GNSS avancées (TST, MST, EOT, TSLV)
 * Nécessite: math.min.js, lib/ephem/vsop2013.js
 */

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

// --- FONCTIONS INTERNES ---

/**
 * Calcule l'Équation du Temps (EOT) en minutes.
 * @param {Date} date - Date et heure actuelles (UTC).
 * @param {number} eclipticLongitude - Longitude écliptique du Soleil (lambda) en degrés, fournie par VSOP2013.
 * @returns {number} EOT en minutes.
 */
function getEOT(date, eclipticLongitude) {
    const lambda = eclipticLongitude * D2R; 
    const epsilon = 23.4392911 * D2R; // Obliquité de l'écliptique
    
    // 1. Calcul de l'Ascension Droite (alpha)
    const tanAlpha = Math.tan(lambda) * Math.cos(epsilon);
    let alpha = Math.atan(tanAlpha);

    // Ajustement des quadrants
    if (Math.sin(lambda) < 0) { alpha += Math.PI; } 
    else if (Math.cos(lambda) < 0) { alpha += Math.PI; }
    alpha = (alpha % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);

    // 2. Calcul de la Longitude Moyenne (L0) - Approximatif J2000
    const J2000 = new Date(Date.UTC(2000, 0, 1, 12, 0, 0));
    const T = (date.getTime() - J2000.getTime()) / (1000 * 60 * 60 * 24 * 36525);
    const L0_deg = 280.46646 + 36000.76983 * T + 0.0003032 * T * T;
    const L0_rad = (L0_deg % 360) * D2R;
    
    // 3. EOT = L0 - alpha (en temps)
    const EOT_rad = L0_rad - alpha;
    
    let EOT_hours = EOT_rad * R2D / 15;
    
    // Normalisation de l'EOT (-0.5h à 0.5h)
    if (EOT_hours > 0.5) EOT_hours -= 24;
    if (EOT_hours < -0.5) EOT_hours += 24;

    const EOT_minutes = EOT_hours * 60;
    
    return EOT_minutes;
}

// --- FONCTIONS EXPORTÉES ---

/**
 * Calcule le Temps Solaire Moyen (MST), le Temps Solaire Vrai (TST) et l'EOT.
 */
function getSolarTime(date, longitudeDeg) {
    const hours = date.getUTCHours();
    const minutes = date.getUTCMinutes();
    const seconds = date.getUTCSeconds();
    const UT_Hours = hours + (minutes / 60) + (seconds / 3600);
    
    let eclipticLongitude = 0;
    try {
        // Ceci utilise la fonction du placeholder VSOP2013.getSolarLongitude
        if (typeof VSOP2013 !== 'undefined' && VSOP2013.getSolarLongitude) {
             eclipticLongitude = VSOP2013.getSolarLongitude(date); 
        } else {
             console.warn("VSOP2013 non disponible. Utilisation de l'approximation de base.");
             const T = (date.getTime() - new Date(Date.UTC(2000, 0, 1, 12, 0, 0)).getTime()) / (1000 * 60 * 60 * 24 * 36525);
             eclipticLongitude = 280.46646 + 36000.76983 * T;
        }
    } catch (e) {
        console.error("Erreur de calcul VSOP2013, vérifiez le fichier vsop2013.js", e);
        return { MST: 'N/A', TST: 'N/A', EOT: 'N/A', solarNoon: null, ECL_LONG: 'N/A' };
    }
    eclipticLongitude = (eclipticLongitude % 360 + 360) % 360; 
    
    const EOT_minutes = getEOT(date, eclipticLongitude);

    // 1. Temps Solaire Moyen (MST)
    let MST_Hours = UT_Hours + (longitudeDeg / 15);
    MST_Hours = (MST_Hours % 24 + 24) % 24; 
    
    // 2. Temps Solaire Vrai (TST)
    let TST_Hours = MST_Hours + (EOT_minutes / 60);
    TST_Hours = (TST_Hours % 24 + 24) % 24; 

    // 3. Midi Solaire (Heure UT de TST = 12:00)
    const solarNoonHoursUT = 12 - (longitudeDeg / 15) - (EOT_minutes / 60);
    const solarNoonTimeMs = date.getTime() + (solarNoonHoursUT * 3600 - UT_Hours * 3600) * 1000;
    const solarNoonDate = new Date(solarNoonTimeMs);
    
    const formatTime = (h) => {
        const hh = Math.floor(h);
        const mm = Math.floor((h - hh) * 60);
        const ss = Math.floor(((h - hh) * 60 - mm) * 60);
        return `${String(hh).padStart(2, '0')}:${String(mm).padStart(2, '0')}:${String(ss).padStart(2, '0')}`;
    };

    return {
        MST: formatTime(MST_Hours),
        TST: formatTime(TST_Hours),
        EOT: EOT_minutes.toFixed(4),
        solarNoon: solarNoonDate,
        ECL_LONG: eclipticLongitude.toFixed(4),
        DateMST: date,
        DateTST: date
    };
}

/**
 * Calcule le Temps Sidéral Local Vrai (TSLV).
 */
function getTSLV(date, longitudeDeg) {
    const JD = date.getTime() / 86400000 + 2440587.5;
    const T = (JD - 2451545.0) / 36525.0; 

    // Temps Sidéral de Greenwich Moyen (GSTM) en heures
    let GSTM_hours = (6.697374558 + 2400.0513369553 * T + 0.000025862 * T * T) % 24;
    if (GSTM_hours < 0) GSTM_hours += 24;

    // Correction UT -> Temps Sidéral
    const UT_hours = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    GSTM_hours += UT_hours * 1.00273790935;

    // Correction d'Équation des Équinoxes (omise, donc GSTV ≈ GSTM)
    const GSTV_hours = GSTM_hours; 

    // Temps Sidéral Local Vrai (TSLV)
    let TSLV_hours = GSTV_hours + (longitudeDeg / 15);
    TSLV_hours = (TSLV_hours % 24 + 24) % 24;

    const hh = Math.floor(TSLV_hours);
    const mm_total = (TSLV_hours - hh) * 60;
    const mm = Math.floor(mm_total);
    const ss = Math.floor((mm_total - mm) * 60);

    return `${String(hh).padStart(2, '0')}:${String(mm).padStart(2, '0')}:${String(ss).padStart(2, '0')}`;
}

window.getSolarTime = getSolarTime;
window.getEOT = getEOT;
window.getTSLV = getTSLV;
