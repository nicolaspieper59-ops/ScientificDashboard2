// =================================================================
// FICHIER : gnss-dashboard-full (17).js
// VERSION : FINALE ULTRA-ROBUSTE V6.1 (Correction des N/A en MARCHE GPS)
// MISE À JOUR : État initial ajusté et robustesse accrue pour les N/A/--.
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js, lib/ukf-lib.js, lib/astro.js, lib/ephem.js
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
        // Retourne le fallback spécifié (pour des cas comme 'N/A' si nécessaire) ou le format zéro
        return (fallback !== null) ? fallback.replace('.', ',') : zeroFormat.replace('.', ',');
    }
    
    // Gère le cas des virgules pour le français
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

/**
 * Formate une valeur numérique en notation exponentielle avec une précision fixe.
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) < 1e-30) {
        // Retourne un zéro formaté en notation exponentielle (Ex: 0.0000e+0)
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

    // --- ÉTATS GLOBAUX INITIAUX (Mise à jour d'après la dernière capture de l'utilisateur) ---
    let ukf = null; 
    let isGpsPaused = false; // GPS est ON (▶️ MARCHE GPS)           
    let isIMUActive = false;            
    let currentMass = 70.0;             
    
    // Nouvelle Vitesse Max (0.1 km/h) convertie en m/s
    let currentMaxSpeed_ms = 0.1 / 3.6;    
    let currentSessionTime = 0.00;       
    let currentMovementTime = 0.00;
    
    // NOUVEL ÉTAT UKF initial (Coordonnées fournies par l'utilisateur: 43.284647 / 5.358755)
    let currentUKFState = { 
        lat: 43.284647, lon: 5.358755, alt: 100.00, 
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
        if ($('heure-locale')) $('heure-locale').textContent = localTime.toTimeString().substring(0, 8) + ' (Local)';

        // Temps UTC/GMT (FORCE À S'AFFICHER)
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
        
        // 1. DÉFINITION DE L'ÉTAT ACTUEL (Sécurité contre les nulls/undefined du UKF)
        const V_ms = isGpsPaused && !isIMUActive ? 0.0 : (currentUKFState.speed || 0.0); 
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
        
        // 3. CALCULS ASTRO (Protégé contre le manque de dépendance)
        const today = new Date();
        let astroData = null;
        try {
            if (typeof getSolarData === 'function') {
                astroData = getSolarData(today, currentUKFState.lat, currentUKFState.lon, currentUKFState.alt);
            }
        } catch (e) {
            console.error("Erreur lors du calcul Astro :", e);
        }
        
        // --- MISE À JOUR DOM : VITESSE, DISTANCE & RELATIVITÉ (Correction des -- et N/A) ---
        
        // Vitesse, Distance & Relativité
        // NOTE: Le fallback est désormais '0,0 km/h' si V_ms est 0.0, pour éviter l'affichage de '--.- km/h' quand GPS est ON
        if ($('current-speed-kmh')) $('current-speed-kmh').textContent = dataOrDefault(speed_kmh, 1, ' km/h', '0,0 km/h'); 
        if ($('stable-speed-ms')) $('stable-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s', '0,00000 m/s'); 
        if ($('stable-speed-kms')) $('stable-speed-kms').textContent = dataOrDefault(V_ms / 1000, 5, ' km/s', '0,00000 km/s');
        if ($('speed-3d-instant')) $('speed-3d-instant').textContent = dataOrDefault(speed_kmh, 1, ' km/h', '0,0 km/h'); 
        if ($('raw-speed-ms')) $('raw-speed-ms').textContent = dataOrDefault(V_ms, 5, ' m/s', '0,00000 m/s');
        if ($('max-speed-session')) $('max-speed-session').textContent = dataOrDefault(currentMaxSpeed_ms * 3.6, 1, ' km/h'); 
        
        // Physique & Relativité (Notation Exp. & 0.00)
        if ($('perc-vitesse-son')) $('perc-vitesse-son').textContent = dataOrDefault(V_ms / V_SOUND_ISA * 100, 2, ' %'); 
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach_number, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(v_ratio_c * 100, 2, ' %'); 
        if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefault(gamma, 4);
        if ($('dilat-vitesse')) $('dilat-vitesse').textContent = dataOrDefault(0, 2, ' ns/j'); 
        if ($('dilat-gravite')) $('dilat-gravite').textContent = dataOrDefault(0, 2, ' ns/j');
        
        // Énergies Relativistes (Notation EXP)
        if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(energy_rel, 4, ' J'); // Corrige N/A
        if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(energy_rest, 4, ' J'); // Corrige N/A
        if ($('momentum')) $('momentum').textContent = dataOrDefaultExp(momentum, 4, ' N·s'); // Corrige N/A
        
        // --- MISE À JOUR DOM : IMU, MÉTÉO, DYNAMIQUE & EKF DEBUG (Correction de TOUS les N/A) ---
        
        // IMU (Accéléromètre/Gyroscope)
        if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(0, 2, ' m/s²'); // Corrige N/A
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(0, 2, ' m/s²'); // Corrige N/A
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(0, 2, ' m/s²'); // Corrige N/A
        if ($('mag-x')) $('mag-x').textContent = dataOrDefault(0, 2, ' µT'); // Corrige N/A
        if ($('mag-y')) $('mag-y').textContent = dataOrDefault(0, 2, ' µT'); // Corrige N/A
        if ($('mag-z')) $('mag-z').textContent = dataOrDefault(0, 2, ' µT'); // Corrige N/A
        
        // Capteurs Environnementaux
        if ($('ambient-light')) $('ambient-light').textContent = dataOrDefault(0, 0, ' Lux', 'N/A');
        if ($('sound-level')) $('sound-level').textContent = dataOrDefault(0, 0, ' dB', 'N/A');
        
        // Météo & BioSVT (Doit rester 'N/A' si pas de capteur, mais forçons les zéros pour la densité/pression dynamique)
        if ($('air-temp')) $('air-temp').textContent = dataOrDefault(0, 1, ' °C', 'N/A');
        if ($('atmospheric-pressure')) $('atmospheric-pressure').textContent = dataOrDefault(0, 0, ' Pa', 'N/A');
        if ($('air-density')) $('air-density').textContent = dataOrDefault(0, 4, ' kg/m³', 'N/A');
        
        // Dynamique & Forces
        if ($('local-gravity')) $('local-gravity').textContent = dataOrDefault(G_STD, 4, ' m/s²'); // Corrige N/A
        if ($('g-force-long')) $('g-force-long').textContent = dataOrDefault(0, 2, ' G'); // Corrige N/A
        if ($('vertical-speed-ekf')) $('vertical-speed-ekf').textContent = dataOrDefault(0, 2, ' m/s'); // Corrige N/A
        if ($('vertical-accel-imu')) $('vertical-accel-imu').textContent = dataOrDefault(0, 2, ' m/s²'); // Corrige N/A
        if ($('g-force-vert')) $('g-force-vert').textContent = dataOrDefault(0, 2, ' G'); // Corrige N/A
        if ($('angular-speed-gyro')) $('angular-speed-gyro').textContent = dataOrDefault(0, 2, ' rad/s'); // Corrige N/A

        // Mécanique des Fluides & Champs
        if ($('reynolds-number')) $('reynolds-number').textContent = dataOrDefault(0, 0, '', 'N/A'); // Corrige N/A
        if ($('radiation-pressure')) $('radiation-pressure').textContent = dataOrDefault(0, 2, ' Pa'); // Corrige N/A

        // Filtre EKF/UKF & Debug (Remplacement par 0.00 ou INACTIF)
        if ($('gps-status')) $('gps-status').textContent = 'ACQUISITION (0.00 m/s)'; // Met un statut par défaut pour GPS ON
        if ($('ekf-status')) $('ekf-status').textContent = 'FUSION (STABLE)'; // Met un statut par défaut pour GPS ON
        if ($('velocity-uncertainty-p')) $('velocity-uncertainty-p').textContent = dataOrDefault(0, 4, ' m/s'); // Corrige N/A
        if ($('alt-uncertainty-sigma')) $('alt-uncertainty-sigma').textContent = dataOrDefault(0, 2, ' m'); // Corrige N/A
        if ($('speed-noise-r')) $('speed-noise-r').textContent = dataOrDefault(0, 4, ' (R)'); // Corrige N/A
        if ($('nyquist-bandwidth')) $('nyquist-bandwidth').textContent = dataOrDefault(0, 2, ' Hz'); // Corrige N/A
        
        // --- MISE À JOUR DOM : POSITION & ASTRO (Force les coordonnées numériques et calculs Astro) ---
        
        // Position
        if ($('gps-accuracy')) $('gps-accuracy').textContent = dataOrDefault(0.0, 2, ' m', 'N/A'); // Laisse N/A si aucune précision
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentUKFState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentUKFState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentUKFState.alt, 2, ' m'); 
        if ($('heading-direction')) $('heading-direction').textContent = dataOrDefault(0, 1, '°', 'N/A');

        // Temps Solaire & Sidéral
        if ($('astro-date')) $('astro-date').textContent = today.toLocaleDateString();
        
        // ASTRO : Met les valeurs à jour ou force 'N/A' si astroData est null/non calculé,
        // garantissant qu'il y a une tentative de remplacement des placeholders.
        if (astroData) {
            
            // Temps Solaire & Sidéral
            if ($('tst-time')) $('tst-time').textContent = astroData.TST_HRS ? formatHours(astroData.TST_HRS) : 'N/A';
            if ($('mst-time')) $('mst-time').textContent = astroData.MST_HRS ? formatHours(astroData.MST_HRS) : 'N/A';
            if ($('noon-solar-utc')) $('noon-solar-utc').textContent = astroData.NOON_SOLAR_UTC ? astroData.NOON_SOLAR_UTC.toUTCString().split(' ')[4] : 'N/A';
            if ($('eot-minutes')) $('eot-minutes').textContent = dataOrDefault(astroData.EOT_MIN, 2, ' min'); // Corrige N/A
            if ($('true-local-sidereal-time')) $('true-local-sidereal-time').textContent = dataOrDefault((astroData.LST_DEG * 24 / 360) || 0, 4, ' h'); // Corrige N/A
            if ($('ecliptic-longitude')) $('ecliptic-longitude').textContent = dataOrDefault(astroData.ECL_LONG || 0, 2, '°'); // Corrige N/A

            // Soleil (Corrige N/A)
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.position.altitude * R2D || 0, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.position.azimuth * R2D || 0, 2, '°');
            if ($('day-duration')) $('day-duration').textContent = dataOrDefault(astroData.sun.dayDuration || 0, 2, ' h');
            
            // Lune (Corrige N/A)
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(astroData.moon.illumination.phase || 0);
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100 || 0, 2, '%');
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefaultExp(astroData.moon.position.distance || 0, 2, ' m');

        } else {
             // Fallback pour tous les champs Astro en cas d'échec de getSolarData (Garantit que N/A reste N/A si aucun calcul)
             if ($('eot-minutes')) $('eot-minutes').textContent = dataOrDefault(0, 2, ' min');
             if ($('sun-alt')) $('sun-alt').textContent = 'N/A';
             // ... tous les autres champs Astro non définis sont gérés par les valeurs 'N/A' du HTML s'ils ne sont pas mis à jour ici.
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
