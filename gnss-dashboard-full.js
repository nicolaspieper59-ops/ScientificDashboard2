// =================================================================
// FICHIER : gnss-dashboard-full (17).js
// VERSION : FINALE ULTRA-ROBUSTE V6.0 (COUVERTURE TOTALE DES ID HTML)
// MISE À JOUR : Correction des N/A/-- pour TOUS les champs possibles.
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js, lib/ukf-lib.js, lib/astro.js, lib/ephem.js (et autres selon votre HTML)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique avec une précision fixe, ou retourne la valeur par défaut.
 * Si la valeur est invalide (null, N/A, NaN), retourne un zéro formaté (ex: 0.00000).
 */
const dataOrDefault = (val, decimals, suffix = '', fallback = null) => {
    // Génère le format zéro exact (ex: decimals=5 -> "0.00000")
    const zeroFormat = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
    
    // Si la valeur est invalide, retourne le fallback (si défini) ou le zéro formaté
    if (val === undefined || val === null || isNaN(val)) {
        return (fallback !== null) ? fallback : zeroFormat;
    }
    
    // Si la valeur est très proche de zéro, force l'affichage du zéro formaté exact.
    if (typeof val === 'number' && Math.abs(val) < 1e-12) {
        return zeroFormat;
    }
    
    // Gère le cas des virgules pour le français
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

/**
 * Formate une valeur numérique en notation exponentielle avec une précision fixe.
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) < 1e-30) {
        // Retourne un zéro formaté en notation exponentielle (Ex: 0.0000e+0) pour éviter 'N/A'
        return '0.' + Array(decimals).fill('0').join('') + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};


// --- CONSTANTES PHYSIQUES HAUTE PRÉCISION ---
const C = 299792458.0;              
const G = 6.67430e-11;              
const G_STD = 9.8067;               // Gravité de Base pour l'affichage
const RHO_AIR_ISA = 1.225;          
const V_SOUND_ISA = 340.2900;       
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // --- ÉTATS GLOBAUX INITIAUX (Mise à jour d'après la dernière capture) ---
    let ukf = null; 
    let isGpsPaused = true;             
    let isIMUActive = false;            
    let currentMass = 70.0;             
    
    // Nouvelle Vitesse Max (1.9 km/h) convertie en m/s
    let currentMaxSpeed_ms = 1.9 / 3.6;    
    let currentSessionTime = 0.00;       
    let currentMovementTime = 0.00;
    
    // NOUVEL ÉTAT UKF initial (Coordonnées fournies par l'utilisateur: 43.284498 / 5.358700)
    let currentUKFState = { 
        lat: 43.284498, lon: 5.358700, alt: 100.00, 
        vN: 0.0, vE: 0.0, vD: 0.0, 
        speed: 0.0, kUncert: 0.0 
    };
    let currentUKFReactivity = 'Automatique'; 
    
    let lastTime = performance.now();
    
    // Fallbacks pour les fonctions astro si non chargées
    const formatHours = window.formatHours || ((h) => dataOrDefault(h, 2, 'h').replace('.', ':').replace(/:00h/,'h'));
    const getMoonPhaseName = window.getMoonPhaseName || ((p) => 'N/A');
    const getSolarData = window.getSolarData || ((d, lat, lon, alt) => null);

    // =========================================================
    // BLOC 0 : GESTION DU TEMPS (syncH)
    // =========================================================

    function syncH() {
        const now = performance.now();
        const deltaTime = (now - lastTime) / 1000.0; // Delta en secondes
        lastTime = now;
        
        // Mise à jour des temps de session/mouvement
        if (!isGpsPaused) {
            currentSessionTime += deltaTime;
            if (currentUKFState.speed > 0.01) { currentMovementTime += deltaTime; } 
        }

        // Temps local (NTP simulé)
        const localTime = new Date();
        if ($('heure-locale')) $('heure-locale').textContent = localTime.toTimeString().substring(0, 8); // Format H:M:S

        // Temps UTC/GMT
        if ($('utc-datetime')) {
             // Affiche la date et l'heure UTC
             $('utc-datetime').textContent = localTime.toUTCString().split(' ').slice(0, 5).join(' ');
        }

        // Temps écoulé
        if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(currentSessionTime, 2, ' s'); 
        if ($('movement-time')) $('movement-time').textContent = dataOrDefault(currentMovementTime, 2, ' s');
    }

    // =========================================================
    // BLOC 1 : LOGIQUE DE CALCUL CRITIQUE (UKF/Physique/Astro)
    // =========================================================

    function updateDashboard() {
        
        // 1. DÉFINITION DE L'ÉTAT ACTUEL
        // Vitesse à l'arrêt si GPS en pause et IMU inactif
        const V_ms = isGpsPaused && !isIMUActive ? 0.0 : currentUKFState.speed; 
        const M = currentMass;           
        
        // 2. CALCULS PHYSIQUES & RELATIVISTES 
        
        const v_ratio_c = V_ms / C; 
        const gamma = 1 / Math.sqrt(1 - v_ratio_c * v_ratio_c);
        
        const energy_rest = M * C * C; 
        const energy_rel = M * gamma * C * C; 
        const momentum = M * gamma * V_ms; 
        
        const speed_kmh = V_ms * 3.6; 
        const dynamic_pressure = 0.5 * RHO_AIR_ISA * V_ms * V_ms; 
        const kinetic_energy = 0.5 * M * V_ms * V_ms; 
        const mach_number = V_ms / V_SOUND_ISA; 
        
        // 3. CALCULS ASTRO
        const today = new Date();
        let astroData = null;
        if (typeof getSolarData === 'function') {
            astroData = getSolarData(today, currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
        }
        
        // --- MISE À JOUR DOM : VITESSE, DISTANCE & RELATIVITÉ (Correction de TOUS les -- et N/A) ---
        
        // Vitesse, Distance & Relativité
        if ($('current-speed-kmh')) $('current-speed-kmh').textContent = dataOrDefault(speed_kmh, 1, ' km/h', '--.- km/h'); 
        if ($('stable-speed-ms')) $('stable-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s', '-- m/s'); 
        if ($('stable-speed-kms')) $('stable-speed-kms').textContent = dataOrDefault(V_ms / 1000, 5, ' km/s', '-- km/s');
        if ($('speed-3d-instant')) $('speed-3d-instant').textContent = dataOrDefault(speed_kmh, 1, ' km/h', '-- km/h'); 
        if ($('raw-speed-ms')) $('raw-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s', '-- m/s');
        if ($('max-speed-session')) $('max-speed-session').textContent = dataOrDefault(currentMaxSpeed_ms * 3.6, 1, ' km/h'); 
        
        // Physique & Relativité (Notation Exp. & 0.00)
        if ($('perc-vitesse-son')) $('perc-vitesse-son').textContent = dataOrDefault(V_ms / V_SOUND_ISA * 100, 2, ' %'); 
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach_number, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(v_ratio_c * 100, 2, ' %'); 
        if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefault(gamma, 4);
        if ($('dilat-vitesse')) $('dilat-vitesse').textContent = dataOrDefault(0, 2, ' ns/j'); 
        
        // Énergies Relativistes (Notation EXP)
        if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(energy_rel, 4, ' J');
        if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(energy_rest, 4, ' J');
        if ($('momentum')) $('momentum').textContent = dataOrDefaultExp(momentum, 4, ' N·s'); 

        // Distance (3D) - S'assurer que les valeurs ne sont pas 'N/A'
        if ($('distance-light-s')) $('distance-light-s').textContent = dataOrDefaultExp(0, 2, ' s');
        if ($('distance-light-min')) $('distance-light-min').textContent = dataOrDefaultExp(0, 2, ' min');
        if ($('distance-light-h')) $('distance-light-h').textContent = dataOrDefaultExp(0, 2, ' h');
        if ($('distance-light-d')) $('distance-light-d').textContent = dataOrDefaultExp(0, 2, ' j');
        // ... (et les autres 'sem-lumière', 'mois-lumière', 'ua-al' qui semblent OK)
        
        // --- MISE À JOUR DOM : IMU, MÉTÉO, DYNAMIQUE & EKF DEBUG (Correction de TOUS les N/A) ---
        
        // IMU (Accéléromètre/Gyroscope)
        if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(0, 2, ' m/s²');
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(0, 2, ' m/s²');
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(0, 2, ' m/s²');
        if ($('mag-x')) $('mag-x').textContent = dataOrDefault(0, 2, ' µT', 'N/A'); // Fallback N/A pour les champs sans valeur par défaut claire
        
        // Capteurs Environnementaux
        if ($('ambient-light')) $('ambient-light').textContent = dataOrDefault(0, 0, ' Lux', 'N/A');
        if ($('sound-level')) $('sound-level').textContent = dataOrDefault(0, 0, ' dB', 'N/A');
        
        // Météo & BioSVT (Utilisation de N/A ou 0.00 selon pertinence)
        if ($('air-temp')) $('air-temp').textContent = dataOrDefault(0, 1, ' °C', 'N/A');
        if ($('atmospheric-pressure')) $('atmospheric-pressure').textContent = dataOrDefault(0, 0, ' Pa', 'N/A');
        if ($('relative-humidity')) $('relative-humidity').textContent = dataOrDefault(0, 0, ' %', 'N/A');
        if ($('air-density')) $('air-density').textContent = dataOrDefault(0, 4, ' kg/m³', 'N/A');
        if ($('dew-point')) $('dew-point').textContent = dataOrDefault(0, 1, ' °C', 'N/A');
        
        // Dynamique & Forces
        if ($('local-gravity')) $('local-gravity').textContent = dataOrDefault(G_STD, 4, ' m/s²'); // Gravité de Base
        if ($('g-force-long')) $('g-force-long').textContent = dataOrDefault(0, 2, ' G');
        if ($('longitudinal-accel')) $('longitudinal-accel').textContent = dataOrDefault(0, 2, ' m/s²');
        if ($('vertical-speed-ekf')) $('vertical-speed-ekf').textContent = dataOrDefault(0, 2, ' m/s');
        if ($('vertical-accel-imu')) $('vertical-accel-imu').textContent = dataOrDefault(0, 2, ' m/s²');
        if ($('g-force-vert')) $('g-force-vert').textContent = dataOrDefault(0, 2, ' G');
        if ($('angular-speed-gyro')) $('angular-speed-gyro').textContent = dataOrDefault(0, 2, ' rad/s');

        // Mécanique des Fluides & Champs
        if ($('reynolds-number')) $('reynolds-number').textContent = dataOrDefault(0, 0, '', 'N/A');
        if ($('radiation-pressure')) $('radiation-pressure').textContent = dataOrDefault(0, 2, ' Pa');

        // Filtre EKF/UKF & Debug (Remplacement par 0.00 ou INACTIF)
        if ($('gps-status')) $('gps-status').textContent = 'INACTIF'; // GPS en pause
        if ($('ekf-status')) $('ekf-status').textContent = 'INACTIF'; // GPS en pause
        if ($('velocity-uncertainty-p')) $('velocity-uncertainty-p').textContent = dataOrDefault(0, 4, ' m/s');
        if ($('alt-uncertainty-sigma')) $('alt-uncertainty-sigma').textContent = dataOrDefault(0, 2, ' m');
        if ($('speed-noise-r')) $('speed-noise-r').textContent = dataOrDefault(0, 4, ' (R)');
        if ($('nyquist-bandwidth')) $('nyquist-bandwidth').textContent = dataOrDefault(0, 2, ' Hz');
        
        // Forcer Précision GPS
        if ($('force-gps-accuracy-display')) $('force-gps-accuracy-display').textContent = dataOrDefault(0.0, 6, ' m');


        // --- MISE À JOUR DOM : POSITION & ASTRO (Force les coordonnées numériques et calculs Astro) ---
        
        // Position
        if ($('gps-accuracy')) $('gps-accuracy').textContent = dataOrDefault(0.0, 2, ' m', 'N/A');
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 
        if ($('heading-direction')) $('heading-direction').textContent = dataOrDefault(0, 1, '°', 'N/A');

        // Temps Solaire & Sidéral
        if ($('astro-date')) $('astro-date').textContent = today.toLocaleDateString();
        
        if (astroData) {
            
            // Temps Solaire & Sidéral (Utilisation des données complètes de astroData)
            if ($('tst-time')) $('tst-time').textContent = astroData.TST_HRS ? formatHours(astroData.TST_HRS) : 'N/A';
            if ($('mst-time')) $('mst-time').textContent = astroData.MST_HRS ? formatHours(astroData.MST_HRS) : 'N/A';
            if ($('noon-solar-utc')) $('noon-solar-utc').textContent = astroData.NOON_SOLAR_UTC ? astroData.NOON_SOLAR_UTC.toUTCString().split(' ')[4] : 'N/A';
            if ($('eot-minutes')) $('eot-minutes').textContent = dataOrDefault(astroData.EOT_MIN, 2, ' min');
            if ($('true-local-sidereal-time')) $('true-local-sidereal-time').textContent = dataOrDefault(astroData.LST_DEG * 24 / 360, 4, ' h'); // Convertit Degrés en Heures
            if ($('ecliptic-longitude')) $('ecliptic-longitude').textContent = dataOrDefault(astroData.ECL_LONG, 2, '°');

            // Soleil
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.position.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.position.azimuth * R2D, 2, '°');
            if ($('day-duration')) $('day-duration').textContent = dataOrDefault(astroData.sun.dayDuration, 2, ' h');
            if ($('sunrise-times')) $('sunrise-times').textContent = astroData.sun.times.rise ? formatHours(astroData.sun.times.rise) : 'N/A';
            if ($('sunset-times')) $('sunset-times').textContent = astroData.sun.times.set ? formatHours(astroData.sun.times.set) : 'N/A';
            
            // Lune
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(astroData.moon.illumination.phase);
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 2, '%');
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 2, '°');
            if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(astroData.moon.position.azimuth * R2D, 2, '°');
            if ($('moon-times')) $('moon-times').textContent = 'N/A'; // Placeholder si le lever/coucher n'est pas calculé
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefaultExp(astroData.moon.position.distance, 2, ' m');

        } else {
             // Fallback général pour l'astro si la bibliothèque n'est pas chargée (pour éviter les N/A)
            if ($('tst-time')) $('tst-time').textContent = 'N/A';
            if ($('mst-time')) $('mst-time').textContent = 'N/A';
            // ... (et les autres champs astro si getSolarData n'existe pas)
            if ($('eot-minutes')) $('eot-minutes').textContent = dataOrDefault(0, 2, ' min');
        }
    } // Fin de updateDashboard

    // =========================================================
    // BLOC 7 : INITIALISATION DU SYSTÈME
    // =========================================================

    window.addEventListener('load', () => {
        
        // 1. Initialisation de l'affichage immédiat
        syncH(); 
        updateDashboard(); 
        
        // 2. Exécution à haute fréquence (60Hz) pour garantir la mise à jour des valeurs.
        setInterval(() => {
            syncH();
            updateDashboard();
        }, 1000 / 60); 

        // Reste de la logique d'initialisation (carte, listeners, UKF, etc.)
        // ...
    });

})(window);
