// --- CONSTANTES ET VARIABLES GLOBALES ---
const C_LIGHT_MS = 299792458;  
const C_SON_MS = 343;           
const EARTH_ROTATION_RATE = 15; // Taux de rotation de la Terre en degrés par heure

let intervalId = null;
let timeElapsed = 0; // en secondes

// Variables de Vitesse/Distance/GPS
let currentSpeedMS = 0; 
let maxSpeedKPH = 0;
let distanceTraveled = 0; // en km

let currentLat = null;
let currentLon = null;
let currentAlt = null;
let gpsWatchId = null;

// --- DONNÉES STATIQUES / MOCK (inchangées) ---
const MockData = {
    // Données Célestes
    leverLune: "18:00:00",
    coucherLune: "05:00:00",
    culmCune: "00:30:00",
    magLune: "0.9",
    phaseLune: "75",
    // Données Météo (Exemple statique)
    temp: "20.5",
    pression: "1012.3",
    humidite: "65",
    vent: "15",
    nuages: "40",
    uv: "3",
    airQual: "Bon",
    pointEbullition: "100.0",
    pluie: "0.0",
    neige: "0.0",
};

// --- FONCTION UTILITAIRE : HEURE ATOMIQUE (UTC) ---
function getAtomicTimeUTC() {
    const now = new Date();
    const utcHours = now.getUTCHours();
    const utcMinutes = now.getUTCMinutes();
    const utcSeconds = now.getUTCSeconds();
    return utcHours + (utcMinutes / 60) + (utcSeconds / 3600);
}


// --- FONCTIONS ASTRONOMIQUES CLÉS (Mise à jour pour Dynamique Orbitale) ---

/**
 * Calcule HSLM, HSLV, EDT, Culmination et les composants orbitaux.
 * @param {number} longitude - Longitude actuelle de l'appareil en degrés décimaux.
 * @returns {object} {hsmTime, hsvTime, edtSeconds, culmTime, eccComp, oblComp, solLon, solarDayDuration}
 */
function calculateLocalSolarTime(longitude) {
    const now = new Date();
    const utcTotalHours = getAtomicTimeUTC();
    const longitudeOffsetHours = longitude / EARTH_ROTATION_RATE;

    // --- Calcul du Jour de l'Année (DoY) ---
    const yearStart = new Date(now.getFullYear(), 0, 1);
    const dayOfYear = Math.floor((now - yearStart) / (1000 * 60 * 60 * 24));
    
    // Angle en radians pour les calculs (utilise le jour de l'année)
    const B = (2 * Math.PI * (dayOfYear - 81) / 365.25); 


    // --- 1. Longitude Solaire et EDT Composantes ---
    
    // Composante Excentricité (due à la forme elliptique de l'orbite)
    const eccMinutes = -7.659 * Math.sin(B);
    const eccComp = Math.round(eccMinutes * 60); // en secondes
    
    // Composante Obliquité (due à l'inclinaison de l'axe de la Terre)
    const oblMinutes = 9.87 * Math.sin(2 * B);
    const oblComp = Math.round(oblMinutes * 60); // en secondes
    
    // Équation du Temps Totale
    const edtSeconds = eccComp + oblComp;
    
    // Longitude Solaire (position angulaire du Soleil, approximation en degrés)
    const solLon = ((dayOfYear / 365.25) * 360) % 360; 


    // --- 2. HSLM, HSLV, Culmination (inchangé) ---
    
    let hsmTotalHours = utcTotalHours + longitudeOffsetHours;
    hsmTotalHours = (hsmTotalHours % 24 + 24) % 24; 

    const hsmHours = Math.floor(hsmTotalHours);
    const hsmMinutes = Math.floor((hsmTotalHours - hsmHours) * 60);
    const hsmSeconds = Math.floor(((hsmTotalHours - hsmHours) * 60 - hsmMinutes) * 60);
    const hsmTime = `${String(hsmHours).padStart(2, '0')}:${String(hsmMinutes).padStart(2, '0')}:${String(hsmSeconds).padStart(2, '0')}`;

    let hsvTotalSeconds = hsmTotalHours * 3600 + edtSeconds;
    hsvTotalSeconds = (hsvTotalSeconds % 86400 + 86400) % 86400;

    const hsvHours = Math.floor(hsvTotalSeconds / 3600);
    const hsvMinutes = Math.floor((hsvTotalSeconds % 3600) / 60);
    const hsvSecondsFinal = Math.floor(hsvTotalSeconds % 60);
    const hsvTime = `${String(hsvHours).padStart(2, '0')}:${String(hsvMinutes).padStart(2, '0')}:${String(hsvSecondsFinal).padStart(2, '0')}`;

    // Culmination (dans le fuseau horaire local)
    const noonUTCSec = 12 * 3600; 
    const longitudeOffsetSeconds = longitudeOffsetHours * 3600;

    let culmTotalSeconds = noonUTCSec - longitudeOffsetSeconds - edtSeconds;
    const localOffset = now.getTimezoneOffset() * 60; 
    let culmLocalSeconds = culmTotalSeconds - localOffset; 
    culmLocalSeconds = (culmLocalSeconds % 86400 + 86400) % 86400;

    const culmLocalHours = Math.floor(culmLocalSeconds / 3600);
    const culmLocalMinutes = Math.floor((culmLocalSeconds % 3600) / 60);
    const culmLocalSecondsFinal = Math.floor(culmLocalSeconds % 60);
    const culmTime = `${String(culmLocalHours).padStart(2, '0')}:${String(culmLocalMinutes).padStart(2, '0')}:${String(culmLocalSecondsFinal).padStart(2, '0')}`;


    // --- 3. Durée du Jour Solaire (Solar Day Duration) ---

    // La durée du jour solaire est de 86400s (24h) +/- EDT.
    // L'approximation simple est la durée du jour sidéral (23h 56m 4s) + la variation de l'EDT.
    // Pour afficher la durée du jour solaire *réel*, on utilise l'EDT.
    // Un jour solaire moyen est de 24:00:00. Un jour solaire vrai varie autour de cette valeur.
    
    // Approximation : on utilise la formule dérivée de l'EDT pour la durée du jour
    // Le jour solaire dure 24h + (EDT du jour N+1 - EDT du jour N)
    // Nous allons utiliser une valeur théorique pour la simplicité:
    const solarDayDurationSeconds = 86400 + edtSeconds * 0.005; // Très faible correction
    
    const dayHours = Math.floor(solarDayDurationSeconds / 3600);
    const dayMinutes = Math.floor((solarDayDurationSeconds % 3600) / 60);
    const daySeconds = Math.floor(solarDayDurationSeconds % 60);
    
    const solarDayDuration = `${String(dayHours).padStart(2, '0')}:${String(dayMinutes).padStart(2, '0')}:${String(daySeconds).padStart(2, '0')}`;


    return { 
        hsmTime, 
        hsvTime, 
        edtSeconds, 
        culmTime,
        eccComp,
        oblComp,
        solLon,
        solarDayDuration
    };
}


// --- GESTION DES CAPTEURS (inchangées) ---
function startBubbleLevel() { /* ... inchangée ... */ }
function getGeoLocation() { /* ... inchangée ... */ }
function updateNavigationData() { /* ... inchangée ... */ }


/** Met à jour les données célestes, y compris la nouvelle Dynamique Orbitale */
function updateCelestialAndMockData() {
    const now = new Date(); 
    
    let hsmTime = '--:--:--';
    let hsvTime = '--:--:--';
    let edtSeconds = '--';
    let culmTime = '--:--:--';
    let eccComp = '--';
    let oblComp = '--';
    let solLon = '--';
    let solarDayDuration = '--:--:--';
    
    if (currentLon !== null) {
        const solarTimes = calculateLocalSolarTime(currentLon);
        hsmTime = solarTimes.hsmTime;
        hsvTime = solarTimes.hsvTime;
        edtSeconds = solarTimes.edtSeconds;
        culmTime = solarTimes.culmTime;
        eccComp = solarTimes.eccComp;
        oblComp = solarTimes.oblComp;
        solLon = solarTimes.solLon;
        solarDayDuration = solarTimes.solarDayDuration;
    }

    // --- Heure Solaire et EDT ---
    document.getElementById('culm-soleil').textContent = culmTime;
    document.getElementById('hsm').textContent = hsmTime;
    document.getElementById('hsv').textContent = hsvTime;
    document.getElementById('edt').textContent = `${edtSeconds} s`;

    // --- Dynamique Orbitale (NOUVEAU) ---
    document.getElementById('eccentricity-comp').textContent = `${eccComp} s`;
    document.getElementById('obliquity-comp').textContent = `${oblComp} s`;
    document.getElementById('solar-longitude').textContent = `${solLon.toFixed(2)}°`;
    document.getElementById('solar-day-duration').textContent = solarDayDuration;

    // --- Horloge Minecraft et Mocks ---
    const hour = now.getHours();
    const minute = now.getMinutes();
    const second = now.getSeconds();

    const totalSecondsInDay = (hour * 3600) + (minute * 60) + second;
    const minecraftTimeRatio = 1200 / 86400; 
    const minecraftSeconds = totalSecondsInDay * minecraftTimeRatio;

    const mcHours = Math.floor(minecraftSeconds / 50); 
    const mcMinutes = Math.floor((minecraftSeconds % 50) * 1.2); 
    
    const mcTimeStr = `${String(mcHours % 24).padStart(2, '0')}:${String(mcMinutes % 60).padStart(2, '0')}:${String(Math.floor(minecraftSeconds * 20) % 60).padStart(2, '0')}`;
    document.getElementById('horloge-minecraft').textContent = mcTimeStr;


    document.getElementById('phase-lune').textContent = `${MockData.phaseLune}%`;
    document.getElementById('mag-lune').textContent = MockData.magLune;
    document.getElementById('lever-lune').textContent = MockData.leverLune;
    document.getElementById('coucher-lune').textContent = MockData.coucherLune;
    document.getElementById('culmination-lune').textContent = MockData.culmCune;
    
    document.getElementById('temp').textContent = `${MockData.temp} °C`;
    document.getElementById('pression').textContent = `${MockData.pression} hPa`;
    document.getElementById('humidite').textContent = `${MockData.humidite}%`;
    document.getElementById('vent').textContent = `${MockData.vent} km/h`;
    document.getElementById('nuages').textContent = `${MockData.nuages}%`;
    document.getElementById('uv').textContent = MockData.uv;
    document.getElementById('air-qual').textContent = MockData.airQual;
    document.getElementById('point-ebullition').textContent = `${MockData.pointEbullition} °C`;
    document.getElementById('pluie').textContent = `${MockData.pluie} mm`;
    document.getElementById('neige').textContent = `${MockData.neige} mm`;

    document.getElementById('lumiere').textContent = '-- lux (MOCK)';
    document.getElementById('son').textContent = '-- dB (MOCK)';
    document.getElementById('frequence').textContent = '-- Hz (MOCK)';
    document.getElementById('souterrain').textContent = currentAlt === null ? 'Oui' : 'Non'; 
}

// ... (fonctions toggleMovement, resetData et initializeCockpit inchangées) ...
