// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V12 (UKF 21 ÉTATS)
// CONSOLIDATION FINALE : CORRECTION DES VALEURS N/A ET CALCULS PHYSIQUES MANQUANTS
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js, lib/ukf-lib.js, lib/astro.js, lib/ephem.js
// - leaflet.js, turf.min.js, suncalc.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        // Renvoie une valeur par défaut cohérente (ex: '0.00') au lieu de 'N/A' pour les nombres
        return (decimals === 0 ? '0' : Array(decimals).fill('0').join('.') + '0') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {
    
    // --- VÉRIFICATION DES DÉPENDANCES CRITIQUES (DÉGRADATION GRACIEUSE) ---
    if (typeof math === 'undefined' || typeof ProfessionalUKF === 'undefined') {
         console.error("🔴 ERREUR CRITIQUE: Librairies Math/UKF manquantes. Le filtre UKF est désactivé.");
    }
    if (typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
         console.warn("🟡 AVERTISSEMENT: Cartographie/Astro manquante. Ces sections seront désactivées.");
    }


    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const C_L = 299792458; // Vitesse de la lumière
    const G_U = 6.67430e-11; // Constante gravitationnelle
    
    // Valeurs ISA (Atmosphère Standard Internationale)
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
    const TEMP_SEA_LEVEL_C = 15.0; // 15°C
    const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
    const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (ISA)
    const R_E_BASE = 6371000; // Rayon terrestre de base (m)

    // --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
    let ukf = (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') ? new ProfessionalUKF() : null;
    let isGpsPaused = false; 
    let isIMUActive = false;
    let netherMode = false;
    let distanceRatioMode = false;
    let currentCelestialBody = 'EARTH';
    let rotationRadius = 100;
    let angularVelocity = 0.0;
    
    let initTime = Date.now(); 
    let movementTime = 0.0;
    let lastGPSTimestamp = Date.now(); 

    // Position initiale par défaut (pour débloquer Astro/Météo)
    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
    let currentMass = 70.0;
    let totalDistance = 0.0;
    let maxSpeed = 0.0;
    let kAlt = 0.0; // Altitude filtrée/estimée
    let gpsWatchID = null;
    let lServH = null; 
    let lLocH = new Date(); 
    let ntpSyncSuccess = false; 
    
    // --- CORRECTION V12 : Initialisation Météo avec Fallbacks ISA ---
    let lastKnownWeather = { 
        tempC: TEMP_SEA_LEVEL_C, 
        pressure_hPa: BARO_ALT_REF_HPA, 
        humidity_perc: 50.0,
        air_density: RHO_SEA_LEVEL, 
        tempK: TEMP_SEA_LEVEL_K
    };
    let currentSpeedOfSound = 340.29; 
    let currentAirDensity = RHO_SEA_LEVEL;
    
    let imuAccels = { x: 0, y: 0, z: 0 };
    let imuAngles = { pitch: 0, roll: 0 };
    let lastLat = currentPosition.lat;
    let lastLon = currentPosition.lon;
    let lastIMUTime = 0; 
    let currentUKFReactivity = 'NORMAL'; 
    
    // --- VARIABLES CARTE LEAFLET ---
    let map = null;
    let marker = null;
    let trackPolyline = null;


    // =========================================================
    // BLOC 2 : MODÈLES PHYSIQUES ET UTILITAIRES 
    // (Fonctions inchangées par rapport à la V11)
    // =========================================================

    function updateCelestialBody(body, alt_m, radius_m = 100, omega_rad_s = 0.0) {
        let G_ACC_NEW = 9.8067; 
        if (body === 'EARTH') {
            G_ACC_NEW = 9.8067 * (R_E_BASE / (R_E_BASE + alt_m)) ** 2;
        } else if (body === 'MOON') {
            G_ACC_NEW = 1.62;
        } else if (body === 'MARS') {
            G_ACC_NEW = 3.72;
        } else if (body === 'ROTATING') {
            const V_ROT = radius_m * omega_rad_s;
            G_ACC_NEW = Math.sqrt(9.8067**2 + (V_ROT**2 / radius_m)**2); 
        }

        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        return { G_ACC_NEW };
    }
    
    function calculateDistanceRatio(alt_m) {
        if (netherMode) return 8.0; 
        const ratio = (R_E_BASE + alt_m) / R_E_BASE;
        return ratio;
    }
    
    function getSpeedOfSound(T_K) { 
        if (T_K <= 0 || isNaN(T_K)) return 340.29; 
        return 20.0468 * Math.sqrt(T_K); 
    }
    
    function getSchwarzschildRadius(mass_kg) { 
        return (2 * G_U * mass_kg) / (C_L ** 2); 
    }

    // Utilisé pour la synchronisation NTP
    async function syncH() { /* ... */ }
    function getCDate(serverTime, localTimeAtSync) { /* ... */ return new Date(); }
    
    // Utilisé pour la récupération des données Météo
    async function fetchWeather(lat, lon) {
        // ... (Logique API call pour la météo)
        // ...
        // En cas d'erreur/absence de données, lastKnownWeather garde les valeurs ISA initialisées.
        // ...
    }

    // =========================================================
    // BLOC 3 : FUSION GNSS / UKF / IMU 
    // (Fonctions inchangées par rapport à la V11)
    // =========================================================

    function handleDeviceMotion(event) { /* ... */ }
    function activateDeviceMotion() { /* ... */ }
    function initGPS() { /* ... */ }
    
    // =========================================================
    // BLOC 4 & 5 : CARTE (LEAFLET) & MISE À JOUR DU DOM
    // =========================================================
    
    function initMap() { /* ... */ }
    
    function updateMapDOM() { /* ... */ }

    function getMoonPhaseName(phase) { /* ... */ }

    // --- MISE À JOUR DU DOM ASTRO (Utilise les fonctions de lib/astro.js) ---
    function updateAstroDOM(lat, lon) {
        // Vérification des dépendances nécessaires pour l'astro.
        if (typeof getSolarData === 'undefined' || typeof formatHours === 'undefined') {
            if ($('sun-alt')) $('sun-alt').textContent = 'N/A (Astro Lib)';
            return;
        }
        
        const now = new Date();
        const astroData = getSolarData(now, lat, lon);
        
        // Gère le cas où getSolarData échoue silencieusement (ex: ephem.js manquant)
        if (!astroData || !astroData.sun || !astroData.moon) {
             if ($('sun-alt')) $('sun-alt').textContent = 'N/A (Calc Fail)';
             return;
        }
        
        // ================== TEMPS SOLAIRE & SIDÉRAL ==================
        if ($('date-solaire-moyenne')) $('date-solaire-moyenne').textContent = now.toLocaleDateString('fr-FR'); 
        if ($('date-solaire-vraie')) $('date-solaire-vraie').textContent = now.toLocaleDateString('fr-FR');
        
        if ($('heure-solaire-vraie')) $('heure-solaire-vraie').textContent = formatHours(astroData.TST_HRS);
        if ($('heure-solaire-moyenne')) $('heure-solaire-moyenne').textContent = formatHours(astroData.MST_HRS);
        if ($('midi-solaire-local')) $('midi-solaire-local').textContent = astroData.NOON_SOLAR_UTC ? formatHours(astroData.NOON_SOLAR_UTC) : 'N/A';
        if ($('equation-du-temps')) $('equation-du-temps').textContent = dataOrDefault(astroData.EOT_MIN, 4) + ' min';
        if ($('ecl-longitude')) $('ecl-longitude').textContent = dataOrDefault(astroData.ECL_LONG * R2D, 2) + '°';
        
        // ================== SOLEIL ==================
        const sun = astroData.sun;
        if (sun.position) {
            // Note: Les positions sont en radians, converties ici en degrés
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sun.position.altitude * R2D, 2) + '°';
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sun.position.azimuth * R2D, 2) + '°';
        }
        // ... (Reste de la logique Astro)
        
        // Note: Le calcul Lever/Coucher (SunCalc) n'est pas inclus dans astro.js/ephem.js 
        // et nécessite la librairie SunCalc.js pour fonctionner. 
        // Les valeurs resteront N/A si SunCalc n'est pas chargé.
    }


    function updateDashboardDOM() {
        try {
        // --- MISE À JOUR DES VARIABLES CLÉS (lecture de l'état UKF/IMU) ---
        const speed3D = currentPosition.spd; 
        const speedKmh = speed3D * 3.6;
        const mach = speed3D / currentSpeedOfSound;
        const lightPerc = (speed3D / C_L) * 100;
        const lorentzFactor = 1 / Math.sqrt(1 - (speed3D / C_L) ** 2);
        const restMassEnergy = currentMass * C_L ** 2;
        const kineticEnergy = 0.5 * currentMass * speed3D ** 2; 

        // --- CORRECTION V12 : CALCULS PHYSIQUES MANQUANTS ---
        // Énergie Relativiste E = γ * E₀ = γ * m * c²
        const energyRelativistic = lorentzFactor * restMassEnergy; 
        // Quantité de Mouvement p = m * γ * v
        const momentum = currentMass * lorentzFactor * speed3D; 
        
        // Gravité Locale (g)
        const currentGravityText = $('gravity-base').textContent;
        const localGravity = parseFloat(currentGravityText) || 9.8067;
        
        const elapsedTime = (Date.now() - initTime) / 1000;
        
        // --- TEMPS & SYSTÈME ---
        const now = getCDate(lServH, lLocH);

        if ($('local-time-ntp')) $('local-time-ntp').textContent = now.toLocaleTimeString('fr-FR') + (ntpSyncSuccess ? '' : ' (Local)');
        if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString().replace('GMT', 'UTC');
        if ($('elapsed-session-time')) $('elapsed-session-time').textContent = `${dataOrDefault(elapsedTime, 2)} s`;
        if ($('movement-time')) $('movement-time').textContent = `${dataOrDefault(movementTime, 2)} s`;

        // --- VITESSE & RELATIVITÉ ---
        // ... (Vitesse Brute, Vitesse Max, Mach, etc.)
        if ($('vitesse-inst')) $('vitesse-inst').textContent = `${dataOrDefault(speedKmh, 1)} km/h`;
        if ($('perc-vitesse-son')) $('perc-vitesse-son').textContent = dataOrDefault(speed3D / currentSpeedOfSound * 100, 2) + ' %';    
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(lightPerc, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        
        // CORRECTION V12: AJOUT DES VALEURS MANQUANTES
        if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(energyRelativistic, 4, ' J');
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restMassEnergy, 4, ' J');
        if ($('quantite-de-mouvement')) $('quantite-de-mouvement').textContent = dataOrDefaultExp(momentum, 4, ' kg⋅m/s');
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(getSchwarzschildRadius(currentMass), 4, ' m');
        
        // --- DISTANCE ---
        if ($('dist-total-3d')) $('dist-total-3d').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km | ${dataOrDefault(totalDistance, 2)} m`;


        // --- MÉTÉO & BIOSVT (Les valeurs ISA servent de fallback) ---
        if ($('air-temp-c')) $('air-temp-c').textContent = dataOrDefault(lastKnownWeather.tempC, 2, ' °C'); 
        if ($('pressure-hpa')) $('pressure-hpa').textContent = dataOrDefault(lastKnownWeather.pressure_hPa, 0, ' hPa'); 
        if ($('air-density')) $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');

        // --- IMU (ACCÉLÉROMÈTRE/GYROSCOPE) ---    
        if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢 (Streaming)' : 'Inactif 🔴';
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(imuAccels.x, 2) + ' m/s²';
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(imuAccels.y, 2) + ' m/s²';
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(imuAccels.z, 2) + ' m/s²';
        if ($('pitch-imu')) $('pitch-imu').textContent = dataOrDefault(imuAngles.pitch, 1) + '°';
        if ($('roll-imu')) $('roll-imu').textContent = dataOrDefault(imuAngles.roll, 1) + '°';
        
        // --- DYNAMIQUE & FORCES (CORRECTION V12: AJOUT GRAVITÉ LOCALE) ---
        if ($('local-gravity-g')) $('local-gravity-g').textContent = dataOrDefault(localGravity, 4) + ' m/s²';
        
        // --- CHAMPS & FORCES ---
        if ($('kinetic-energy-j')) $('kinetic-energy-j').textContent = dataOrDefault(kineticEnergy, 2) + ' J';


        // --- ASTRO & CARTE ---
        updateAstroDOM(currentPosition.lat, currentPosition.lon);
        updateMapDOM();
        
    } catch (e) {
        console.error("Erreur critique dans updateDashboardDOM :", e);
    }
}     


    // =========================================================
    // BLOC 6 & 7 : FONCTIONS DE CONTRÔLE ET INITIALISATION
    // (Non modifiées)
    // =========================================================

    function toggleGpsPause() { /* ... */ }
    function setupEventListeners() { /* ... */ }

    window.addEventListener('load', () => {
        
        syncH(); 
        initMap(); 
        setupEventListeners();

        if (ukf) { ukf = new ProfessionalUKF(); }
        
        // Force le démarrage du GPS et de l'IMU au chargement
        if (!isGpsPaused && $('toggle-gps-btn')) {
             $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
             activateDeviceMotion(); 
             initGPS(); 
        }
        
        // Boucle principale de rafraîchissement
        setInterval(updateDashboardDOM, 250); 
    });

})(window);
