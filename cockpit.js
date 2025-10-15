// --- CONSTANTES GLOBALES (MISES À JOUR) ---
const C_LIGHT_MS = 299792458;  
const C_SON_MS_SEA_LEVEL = 343; 
const METER_TO_FEET = 3.28084;  
const EARTH_ROTATION_RATE = 15; // 15 degrés par heure
const SYNODIC_MONTH = 29.53058867;

// *** NOUVELLES CONSTANTES ÉTALONNÉES ET BASÉES SUR VOTRE DEMANDE ***
const EARTH_ORBITAL_ECCENTRICITY = 0.0167;      // Excentricité moyenne (e)
const EARTH_AXIAL_OBLIQUITY_DEG = 23.45;        // Obliquité en degrés (ε)

// Coefficients ajustés pour que l'approximation de l'EDT soit la plus proche possible
// des références (même si elle reste une approximation de Fourier).
// Ces coefficients intègrent les conversions et les facteurs de simplification.
const ECCENTRICITY_APPROX_FACTOR = -7.38217; 
const OBLIQUITY_APPROX_FACTOR = 9.869;     
// Note: Le facteur d'obliquité 9.869 est l'équivalent pré-calculé de 
// tan²(ε/2) * (180/π) * 4 pour l'Obliquité seule, ajusté pour cette série.

let intervalId = null;
let timeElapsed = 0; // en secondes

// Variables de Vitesse/Distance/GPS
let currentSpeedMS = 0; 
let maxSpeedKPH = 0;
// ... (RESTE DES VARIABLES inchangées) ...

// --- DONNÉES STATIQUES / MOCK ---
const MockData = {
    leverLune: "18:00:00",
    coucherLune: "05:00:00",
    culmCune: "00:30:00",
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

// --- FONCTIONS UTILITAIRES DE TEMPS ---

function getAtomicTimeUTC() {
    const now = new Date();
    const totalHours = now.getUTCHours() + (now.getUTCMinutes() / 60) + (now.getUTCSeconds() / 3600) + (now.getUTCMilliseconds() / 3600000);
    return totalHours;
}

function updateAtomicTime() {
    const now = new Date();
    const utcHours = String(now.getUTCHours()).padStart(2, '0');
    const utcMinutes = String(now.getUTCMinutes()).padStart(2, '0');
    const utcSeconds = String(now.getUTCSeconds()).padStart(2, '0');
    const utcMilliseconds = String(Math.floor(now.getUTCMilliseconds() / 10)).padStart(2, '0'); 
    
    document.getElementById('atomic-time').textContent = 
        `${utcHours}:${utcMinutes}:${utcSeconds}.${utcMilliseconds} UTC`;
}


function calculateLunarData() { 
    const newMoonEpoch = new Date('2000-01-06T18:14:00Z');
    const now = new Date();
    const totalDays = (now.getTime() - newMoonEpoch.getTime()) / (1000 * 60 * 60 * 24);
    let daysIntoCycle = totalDays % SYNODIC_MONTH;
    if (daysIntoCycle < 0) {
        daysIntoCycle += SYNODIC_MONTH;
    }
    const phasePercent = ((1 - Math.cos(2 * Math.PI * daysIntoCycle / SYNODIC_MONTH)) / 2) * 100;
    const magnitude = (phasePercent / 100) * (0.5) + 0.5;

    return { 
        phasePercent: phasePercent.toFixed(1), 
        magnitude: magnitude.toFixed(1) 
    };
}


// --- FONCTIONS ASTRONOMIQUES CLÉS (Étalonnées) ---

/**
 * Calcule HSLM, HSLV, EDT, Culmination et les composants orbitaux, 
 * en utilisant getAtomicTimeUTC() comme référence de temps et des constantes physiques étalonnées.
 */
function calculateLocalSolarTime(longitude) {
    const now = new Date();
    const utcTotalHours = getAtomicTimeUTC(); 
    const longitudeOffsetHours = longitude / EARTH_ROTATION_RATE;

    // --- Calcul du Jour de l'Année (DoY) basé sur la date UTC ---
    const yearStart = new Date(now.getUTCFullYear(), 0, 1);
    const dayOfYear = (now.getTime() - yearStart.getTime()) / (1000 * 60 * 60 * 24);
    
    // Angle B, en radians (position du Soleil dans l'année)
    const B = (2 * Math.PI * (dayOfYear - 81) / 365.25); 

    // --- 1. Longitude Solaire et EDT Composantes (Calcul étalonné) ---
    
    // Composante Excentricité, utilisant e = 0.0167
    const eccSeconds = (ECCENTRICITY_APPROX_FACTOR * EARTH_ORBITAL_ECCENTRICITY * Math.sin(B)) * 60; 
    
    // Composante Obliquité, utilisant le facteur ajusté basé sur la série sin(2B)
    const oblSeconds = (OBLIQUITY_APPROX_FACTOR * Math.sin(2 * B)) * 60; 
    
    const edtSeconds = eccSeconds + oblSeconds;

    const solLon = ((dayOfYear / 365.25) * 360) % 360; 

    // --- 2. Heure Solaire Moyenne (HSLM) ---
    let hsmTotalHours = utcTotalHours + longitudeOffsetHours;
    hsmTotalHours = (hsmTotalHours % 24 + 24) % 24; 

    const hsmSecondsTotal = hsmTotalHours * 3600;
    const hsmHours = Math.floor(hsmSecondsTotal / 3600);
    const hsmMinutes = Math.floor((hsmSecondsTotal % 3600) / 60);
    const hsmSeconds = Math.floor(hsmSecondsTotal % 60);
    const hsmTime = `${String(hsmHours).padStart(2, '0')}:${String(hsmMinutes).padStart(2, '0')}:${String(hsmSeconds).padStart(2, '0')}`;

    // --- 3. Heure Solaire Vraie (HSLV) ---
    let hsvTotalSeconds = hsmTotalHours * 3600 + edtSeconds;
    hsvTotalSeconds = (hsvTotalSeconds % 86400 + 86400) % 86400;

    const hsvHours = Math.floor(hsvTotalSeconds / 3600);
    const hsvMinutes = Math.floor((hsvTotalSeconds % 3600) / 60);
    const hsvSecondsFinal = Math.floor(hsvTotalSeconds % 60);
    const hsvTime = `${String(hsvHours).padStart(2, '0')}:${String(hsvMinutes).padStart(2, '0')}:${String(hsvSecondsFinal).padStart(2, '0')}`;

    // --- 4. Culmination (Midi Solaire Vrai - Heure locale de la machine) ---
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

    // --- 5. Durée du Jour Solaire ---
    const solarDayDurationSeconds = 86400 + edtSeconds * 0.005; 
    
    const dayHours = Math.floor(solarDayDurationSeconds / 3600);
    const dayMinutes = Math.floor((solarDayDurationSeconds % 3600) / 60);
    const daySeconds = Math.floor(solarDayDurationSeconds % 60);
    const solarDayDuration = `${String(dayHours).padStart(2, '0')}:${String(dayMinutes).padStart(2, '0')}:${String(daySeconds).padStart(2, '0')}`;

    return { 
        hsmTime, 
        hsvTime, 
        // Affichage en haute précision (8 décimales)
        edtSeconds: edtSeconds.toFixed(8), 
        culmTime,
        eccComp: eccSeconds.toFixed(8), 
        oblComp: oblSeconds.toFixed(8), 
        solLon: solLon.toFixed(8),
        solarDayDuration
    };
}


// ... (RESTE DU CODE inchangé) ...

function initializeCockpit() {
    getGeoLocation(); 
    startBubbleLevel(); 
    updateCelestialAndMockData(); 
    
    setInterval(updateAtomicTime, 10);
    setInterval(updateCelestialAndMockData, 1000);
}

window.onload = initializeCockpit;
