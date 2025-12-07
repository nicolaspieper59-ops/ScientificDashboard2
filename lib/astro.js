// =================================================================
// FICHIER : lib/astro.js
// VERSION : ROBUSTE ET COMPLÈTE (Compatible gnss-dashboard-full)
// DÉPENDANCE : suncalc.js
// =================================================================

(function(window) {

    // --- CONSTANTES ---
    const R2D = 180 / Math.PI; 
    const D2R = Math.PI / 180; 
    const J1970 = 2440588;
    const J2000 = 2451545.0; 
    const dayMs = 1000 * 60 * 60 * 24;
    const MC_DAY_MS = 72 * 60 * 1000; // Durée jour Minecraft

    // --- FONCTIONS UTILITAIRES DE CALENDRIER ---

    function toDays(date) { 
        return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; 
    }

    function solarMeanAnomaly(d) { 
        return D2R * (356.0470 + 0.9856002585 * d); 
    }

    function eclipticLongitude(M) {
        var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
            P = D2R * 102.9377;                                                                
        return M + C + P + Math.PI;
    }

    /**
     * Calcule le Temps Solaire Vrai (TST), Moyen (MST) et l'Équation du Temps.
     * Cette version est alignée avec les objets Date retournés par le Dashboard.
     */
    function getSolarTime(date, lon) {
        if (!date || lon === null || isNaN(lon) || typeof SunCalc === 'undefined') {
            return { TST: 'N/A', MST: 'N/A', EOT: 'N/A', ECL_LONG: 'N/A', DateMST: null, DateTST: null };
        }
        
        const d = toDays(date);
        const M = solarMeanAnomaly(d); 
        const L = eclipticLongitude(M); 
        
        // Utilisation de SunCalc pour une précision de l'EOT
        const eot_min = SunCalc.getEquationOfTime(date) * 60; 

        const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
        const mst_offset_ms = lon * dayMs / 360; 
        const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;
        const eot_ms = eot_min * 60000;
        const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 

        const toTimeString = (ms) => {
            let h = Math.floor(ms / 3600000);
            let m = Math.floor((ms % 3600000) / 60000);
            let s = Math.floor((ms % 60000) / 1000);
            return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
        };

        return { 
            TST: toTimeString(tst_ms), 
            MST: toTimeString(mst_ms), 
            EOT: eot_min.toFixed(2),
            ECL_LONG: (L * R2D).toFixed(2),
            // Objets Date utiles pour l'affichage dans l'interface
            DateMST: new Date(date.getTime() + mst_offset_ms),
            DateTST: new Date(date.getTime() + mst_offset_ms + eot_ms)
        };
    }

    /**
     * Calcule le Temps "Minecraft" (Cycle accéléré de 72 minutes).
     */
    function getMinecraftTime(date) {
        if (date === null) return '00:00';
        const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
        const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
        const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
        
        let h = Math.floor(mcTimeMs / 3600000);
        let m = Math.floor((mcTimeMs % 3600000) / 60000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;
    }

    /**
     * Retourne le nom de la phase lunaire avec Émoji (Format Dashboard).
     */
    function getMoonPhaseName(phase) {
        if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
        if (phase < 0.23) return "Premier Croissant 🌒";
        if (phase < 0.27) return "Premier Quartier 🌓";
        if (phase < 0.48) return "Gibbeuse Croissante 🌔";
        if (phase < 0.52) return "Pleine Lune 🌕";
        if (phase < 0.73) return "Gibbeuse Décroissante 🌖";
        if (phase < 0.77) return "Dernier Quartier 🌗";
        return "Dernier Croissant 🌘"; 
    }

    /**
     * Retourne le Temps Sidéral Local Vrai (TSLV) en format heure décimale.
     */
    function getTSLV(date, lon) {
        if (date === null || isNaN(lon)) return 'N/A';
        const GMST = (date.getUTCHours() + date.getUTCMinutes() / 60) * 15; 
        const LST = GMST + lon;
        const LST_h = (LST / 15 + 24) % 24;
        return LST_h.toFixed(2) + ' h';
    }

    // --- EXPOSITION GLOBALE ---
    window.getSolarTime = getSolarTime;
    window.getMinecraftTime = getMinecraftTime;
    window.getMoonPhaseName = getMoonPhaseName;
    window.getTSLV = getTSLV;
    
    // Fonctionnalité bonus : Calcul haute précision (Wrapper SunCalc)
    window.calculateAstroDataHighPrec = function(date, lat, lon) {
        if (typeof SunCalc === 'undefined') return null;
        const sunPos = SunCalc.getPosition(date, lat, lon);
        const moonPos = SunCalc.getMoonPosition(date, lat, lon);
        return { sun: sunPos, moon: moonPos };
    };

})(window);
