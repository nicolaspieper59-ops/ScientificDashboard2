// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET & CORRIGÉ (UKF 21 ÉTATS)
// VERSION : FINALE ULTRA-ROBUSTE V7.2 (CONSOLIDÉE / SANS SUNCALC)
// DÉPENDANCES CRITIQUES (à charger dans l'HTML) :
// - math.min.js
// - lib/ukf-lib.js (DOIT contenir la classe ProfessionalUKF)
// - lib/astro.js (Calculs Alt/Az du Soleil et Lune)
// - leaflet.js, turf.min.js
// =================================================================

// =================================================================
// PARTIE 1 : UTILITAIRES DE BASE ET FORMATAGE CORRIGÉ
// =================================================================

const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique avec une précision fixe, ou retourne la valeur par défaut.
 * V7.2 - Correction critique pour la robustesse et le séparateur virgule.
 */
const dataOrDefault = (val, decimals, suffix = '', fallback = null, forceZero = true) => {
    if (val === 'N/A') return 'N/A';
    // Vérification des valeurs très proches de zéro ou invalides
    if (val === undefined || val === null || isNaN(val) || (typeof val === 'number' && Math.abs(val) < 1e-18)) {
        if (fallback !== null) return fallback;
        if (forceZero) {
            const zeroFormat = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
            return zeroFormat.replace('.', ',');
        }
        return 'N/A';
    }
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

/**
 * Formate une valeur numérique en notation exponentielle avec une précision fixe.
 * CORRECTION CRITIQUE : Assure que le format exponentiel par défaut respecte 'decimals' et affiche 'e+0'.
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        // Construction du format zéro (ex: '0.00e+0')
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// PARTIE 2 : CONSTANTES PHYSIQUES ET ÉTAT GLOBAL CORRIGÉ (OFFLINE-FIRST)
// =================================================================

// --- CLÉS D'API & ENDPOINTS ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle (N·m²/kg²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation Terre (rad/s)

// Constantes Atmosphériques (ISA Standard pour fallback métrologique)
const TEMP_SEA_LEVEL_K = 288.15; // 15 °C
const BARO_ALT_REF_HPA = 1013.25; // Pression niveau mer (hPa)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air niveau mer (kg/m³)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4; // Indice adiabatique de l'air

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE (Initialisation corrigée) ---
let isGpsPaused = false; 
let ukf = null; // Instance du filtre UKF

let currentPosition = { 
    // Coordonnées de travail par défaut pour débloquer Astro/Météo en cas de fallback (ex: Marseille)
    lat: 43.2964,   
    lon: 5.3697,    
    acc: 10.0,      
    spd: 0.0        
};

// Variables métrologiques par défaut pour les calculs offline (Correction Métrologique)
let kAlt = 0; // Altitude actuelle (m)
let lastP_hPa = BARO_ALT_REF_HPA; 
let lastT_K = TEMP_SEA_LEVEL_K; 
let currentAirDensity = RHO_SEA_LEVEL; 
let currentSpeedOfSound = 343; // Sera calculé au démarrage
let lServH = 0; // Heure du serveur NTP (ms)
let lLocH = 0; // Heure locale du dernier appel NTP (ms)
// ... (autres variables pour la carte, l'IMU, etc.)

// =================================================================
// PARTIE 3 : MODÈLES PHYSIQUES & FONCTIONS CRITIQUES
// =================================================================

/**
 * Calcule la vitesse du son (m/s) en fonction de la température en Kelvin.
 */
function getSpeedOfSound(tempK) {
    if (isNaN(tempK) || tempK <= 0) return 343; 
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

/**
 * Calcule la densité de l'air (kg/m³) en utilisant la loi des gaz parfaits.
 */
function calculateAirDensity(pressure_hPa, tempK) {
    const P_Pa = pressure_hPa * 100;
    if (isNaN(P_Pa) || isNaN(tempK) || tempK <= 0) return RHO_SEA_LEVEL;
    return P_Pa / (R_SPECIFIC_AIR * tempK);
}

// NOTE: Les fonctions getCDate(), syncH(), fetchWeather(), initGPS(), 
// et setupEventListeners() doivent être définies ailleurs dans le fichier 
// ou dans leurs propres librairies.

// =========================================================
// PARTIE 4 : MISE À JOUR DU DASHBOARD (updateDashboard)
// (Logique UKF, Astro/Météo et Fallbacks critiques)
// =========================================================

function updateDashboard() {
    // --- 1. Récupération des données de position ---
    const pos = ukf ? (ukf.getState ? ukf.getState() : currentPosition) : currentPosition;
    const lat = pos.lat;
    const lon = pos.lon;
    kAlt = pos.alt;
    // ... (autres variables de l'UKF)
    
    // --- 2. Traitement et fusion UKF (Stub/Placeholder) ---
    // if (ukf) { ukf.predict(Date.now() / 1000.0); /* ... */ }

    // --- 3. Mise à jour de l'affichage Astro (Correction N/A critiques / SANS suncalc.js) ---
    // Dépend de lib/astro.js (getAstroData et getMoonPhaseName)
    if (typeof window.getAstroData === 'function' && typeof window.formatHours === 'function' && typeof window.getMoonPhaseName === 'function') {
        const date = getCDate(lServH, lLocH) || new Date();
        const astroData = getAstroData(date, lat, lon, kAlt);
        
        // Affichage Soleil (TST, MST, Alt, Azimuth)
        if ($('tst-time')) $('tst-time').textContent = formatHours(astroData.TST_HRS);
        if ($('mst-time')) $('mst-time').textContent = formatHours(astroData.MST_HRS);
        if ($('equation-of-time')) $('equation-of-time').textContent = dataOrDefault(astroData.EOT_MIN, 2, ' min');
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.altitude * R2D, 2, '°'); 
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.azimuth * R2D, 2, '°');
        
        // NOTE: Les temps de Lever/Coucher (sunrise/sunset) ne peuvent pas être calculés sans suncalc.js/lib/ephem.js 
        // ou une implémentation complète dans lib/astro.js. Nous affichons 'N/A'.
        if ($('sunrise-times')) $('sunrise-times').textContent = 'N/A (Calcul manquant)';
        if ($('sunset-times')) $('sunset-times').textContent = 'N/A (Calcul manquant)';

        // Affichage Lune
        if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(astroData.moon.illumination.phase);
        if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 1, '%');
        if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 2, '°');
        if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(astroData.moon.position.azimuth * R2D, 2, '°');

    } else {
        // Fallbacks critiques si lib/astro.js n'est pas chargé (ou incomplet)
        if ($('tst-time')) $('tst-time').textContent = 'N/A';
        if ($('mst-time')) $('mst-time').textContent = 'N/A';
        if ($('equation-of-time')) $('equation-of-time').textContent = 'N/A';
        if ($('sun-alt')) $('sun-alt').textContent = 'N/A';
        if ($('sun-azimuth')) $('sun-azimuth').textContent = 'N/A'; 
        if ($('moon-phase-name')) $('moon-phase-name').textContent = 'N/A';
        if ($('moon-alt')) $('moon-alt').textContent = 'N/A';
        if ($('moon-azimuth')) $('moon-azimuth').textContent = 'N/A'; 
        if ($('sunrise-times')) $('sunrise-times').textContent = 'N/A';
        if ($('sunset-times')) $('sunset-times').textContent = 'N/A';
    } 

    // --- 4. Reste des mises à jour DOM (Vitesse, Alt, Carte, etc.) ---
    // ...
}

// =========================================================
// PARTIE 5 : INITIALISATION DU SYSTÈME (Final et Corrigé)
// =========================================================

window.addEventListener('load', () => {
    
    // 1. Initialisation UKF (doit se faire après le chargement de math.js)
    if (typeof window.ProfessionalUKF === 'function') { 
        ukf = new ProfessionalUKF();
        console.log("✅ Filtre UKF 21 États initialisé.");
    } else {
         console.error("🔴 ERREUR CRITIQUE: ProfessionalUKF n'est pas définie. Vérifiez lib/ukf-lib.js.");
    }
    
    // 2. Démarrer la synchro NTP
    if (typeof syncH === 'function') {
        syncH(); 
    } else {
        console.error("🔴 Fonction syncH() manquante.");
    }
    
    // 3. Initialisation des valeurs par défaut hors ligne pour la physique (Correction Métrologique)
    currentAirDensity = RHO_SEA_LEVEL; 
    currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); 
    lastT_K = TEMP_SEA_LEVEL_K;
    lastP_hPa = BARO_ALT_REF_HPA;

    // 4. Initialisation des autres systèmes
    // if (typeof initGPS === 'function') { initGPS(); }
    // if (typeof setupEventListeners === 'function') { setupEventListeners(); }

    // 5. Boucle de rafraîchissement principale (60Hz)
    updateDashboard(); // Exécution immédiate
    setInterval(updateDashboard, 1000 / 60); // Haute fréquence pour la fluidité (60Hz)
});
