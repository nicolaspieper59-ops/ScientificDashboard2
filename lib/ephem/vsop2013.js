/**
 * lib/ephem/vsop2013.js - Placeholder Fonctionnel
 * * ATTENTION: Ce fichier contient des FORMULES D'APPROXIMATION ASTRONOMIQUE 
 * (basées sur Meeus/J2000) et non les données brutes du modèle VSOP2013.
 * Il sert à satisfaire la dépendance et à fournir une précision raisonnable pour un tableau de bord.
 */
const VSOP2013 = {
    
    /**
     * Calcule la longitude écliptique du Soleil (lambda) en degrés.
     * @param {Date} date - Date et heure (UTC).
     * @returns {number} Longitude écliptique en degrés.
     */
    getSolarLongitude: function(date) {
        // 1. Calcul du temps T en siècles juliens depuis J2000
        const J2000 = new Date(Date.UTC(2000, 0, 1, 12, 0, 0));
        const deltaT = (date.getTime() - J2000.getTime()) / 86400000; 
        const T = deltaT / 36525; 

        // 2. Calcul des variables de base (Longitude moyenne L, Anomalie moyenne g)
        
        // Longitude moyenne du Soleil (L) en degrés
        let L = 280.46646 + 36000.76983 * T + 0.0003032 * T * T;
        L = L % 360;
        
        // Anomalie moyenne du Soleil (g) en degrés
        let g = 357.52911 + 35999.05034 * T - 0.0001559 * T * T - 0.00000048 * T * T * T;
        g = g % 360;

        const gRad = g * (Math.PI / 180);
        
        // 3. Équation du centre (C)
        // La série de coefficients pour C (pour ajuster la vitesse orbitale)
        const C = (1.914602 - 0.004817 * T - 0.000014 * T * T) * Math.sin(gRad) + 
                  (0.019993 - 0.000101 * T) * Math.sin(2 * gRad) + 
                  0.000289 * Math.sin(3 * gRad);

        // 4. Longitude Vraie (lambda)
        let lambda = L + C;
        return (lambda % 360 + 360) % 360;
    }
};

window.VSOP2013 = VSOP2013;
