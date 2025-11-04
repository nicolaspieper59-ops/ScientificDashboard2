// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes & Kalman)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES (Physiques, GPS, Temps) ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s)
const G_ACC = 9.80665; // Gravité standard (m/s²)
const KMH_MS = 3.6; 
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const W_EARTH = 7.2921E-5; // Vitesse angulaire de la Terre (rad/s)
const NETHER_RATIO = 1 / 8; 

// Constantes Temps / Astro
const dayMs = 86400000;
const J1970 = 2440588; 
const J2000 = 2451545; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// Constantes GPS
const MIN_DT = 0.05; 
const MIN_SPD = 0.01; 
const MAX_ACC = 20; 
const GOOD_ACC_THRESHOLD = 3.0; // Seuil de précision GPS (m) où l'on réduit le bruit IMU
const ALT_TH = -50; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman (Vitesse 3D & Altitude)
const Q_NOISE = 0.001; // Bruit de processus Vitesse
const Q_ALT_NOISE = 0.0005; // Bruit de processus Altitude
let kSpd = 0; // Estimation vitesse (m/s)
let kUncert = 1000; // Incertitude vitesse (m/s)²
let kAlt = 0; // Estimation altitude (m)
let kAltUncert = 1000; // Incertitude altitude (m)²
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};

// Constantes ZVU / IMU-Only / Bruit
const R_GPS_DISABLED = 100000.0; // R élevé pour désactiver mathématiquement le GPS (Mode IMU-Only)
const VEL_NOISE_FACTOR = 10; // Utilisé pour seuil dynamique ZVU : acc/VEL_NOISE_FACTOR
const IMU_NOISE_FLOOR = 0.05; // Bruit électronique typique des accéléromètres (m/s²)
const ZVU_SAFETY_MARGIN = 0.15; // Marge pour dépasser le bruit réel
const STATIC_ACCEL_THRESHOLD = IMU_NOISE_FLOOR + ZVU_SAFETY_MARGIN; // CALCULÉ : ~0.2 m/s²
const LOW_SPEED_THRESHOLD = 1.0; // Seuil de vitesse (m/s) pour amortissement dynamique de R
const ACCEL_FILTER_ALPHA = 0.8; 
const ACCEL_MOVEMENT_THRESHOLD = 0.5; 
let kAccel = { x: 0, y: 0, z: 0 };
let G_STATIC_REF = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; // Accélération verticale pour filtre altitude
let latestLinearAccelMagnitude = 0; // Magnitude de l'accélération pour contrôle EKF

// --- VARIABLES GLOBALES (État du système) ---
let wID = null;
let map = null;
let marker = null;
let tracePolyline = null;
let lat = 0, lon = 0;
let sTime = null; 
let distM = 0; 
let timeMoving = 0; 
let maxSpd = 0;
let lastFSpeed = 0;
let currentGPSMode = 'HIGH_FREQ';
let lPos = null; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL';
let lServH = 0; 
let lLocH = 0; 
let domID = null; 
let weatherID = null; 
let lastP_hPa = 1013.25; 

// --- CONSTANTES API MÉTÉO ---
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // <-- REMPLACEZ CECI
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather"; 


// ===========================================
// FONCTIONS UTILITAIRES ET KALMAN
// ===========================================

/** Calcule la distance de Haversine entre deux points (m). */
function dist(lat1, lon1, lat2, lon2) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_E * c;
}

/** Filtre de Kalman 1D (Vitesse) avec entrée de contrôle Accélération IMU */
function kFilter(z, dt, R_dyn, u_accel = 0) {
    // 1. Prediction : Utilisation de l'accélération IMU (u_accel) comme entrée de contrôle
    const predSpd = kSpd + u_accel * dt; 
    
    // Le bruit de processus (Q_NOISE) augmente avec le temps (dt)
    const predUncert = kUncert + Q_NOISE * dt;
    
    // 2. Mesure (Correction par le GPS)
    const K = predUncert / (predUncert + R_dyn);
    kSpd = predSpd + K * (z - predSpd);
    kUncert = (1 - K) * predUncert;
    return kSpd;
}

/** Calcule le bruit de mesure dynamique R (Intègre la Vitesse EKF/Énergie Cinétique) */
function getKalmanR(acc, alt, pressure) { 
    let R_raw = acc * acc; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    const MASS_PROXY = 0.05; 

    // 1. Facteur d'Environnement (Noise Multiplier)
    let noiseMultiplier = envFactor;
    if (alt !== null && alt < 0) {
        noiseMultiplier += Math.abs(alt / 100); 
    }
    
    // 2. Facteur de Vitesse/Énergie Cinétique (Réduction du bruit R aux hautes vitesses)
    const kSpd_squared = kSpd * kSpd;
    const speedFactor = 1 / (1 + MASS_PROXY * kSpd_squared); // Réduit R aux hautes vitesses

    // 3. Boost R aux basses vitesses (non-ZVU) pour fluidifier les mouvements lents.
    let lowSpeedBoost = 1.0;
    if (kSpd > MIN_SPD && kSpd < LOW_SPEED_THRESHOLD) {
        // Interpolation de 1.0 (à LOW_SPEED_THRESHOLD) à 2.0 (vers 0 m/s)
        const speed_ratio = (LOW_SPEED_THRESHOLD - kSpd) / LOW_SPEED_THRESHOLD; 
        lowSpeedBoost = 1.0 + speed_ratio * 1.0; // Boost R jusqu'à 2x à la vitesse minimale
    }
    
    // R final
    let R_dyn = R_raw * noiseMultiplier * speedFactor * lowSpeedBoost; // Application du boost
    
    // S'assurer que R n'est jamais trop petit
    R_dyn = Math.max(R_dyn, 0.01); 

    return R_dyn;
}

/** Filtre de Kalman 1D pour l'Altitude (Utilise u_accel IMU) */
function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    if (z === null) return kAlt; 

    // 1. Prediction (avec Accélération IMU comme entrée de contrôle)
    const predAlt = kAlt + (0.5 * u_accel * dt * dt); 
    let predAltUncert = kAltUncert + Q_ALT_NOISE * dt;

    // 2. Mesure (R_alt est basé sur la précision brute du GPS)
    const R_alt = acc * acc * 2.0; 
    const K = predAltUncert / (predAltUncert + R_alt);
    kAlt = predAlt + K * (z - predAlt);
    kAltUncert = (1 - K) * predAltUncert;

    return kAlt;
}

/** Calcule le point de rosée (utilitaire pour la météo) */
function calculateDewPoint(tempC, humidity) {
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
}

/** Calcule le nom de la phase de la Lune à partir du coefficient de phase (0.0 à 1.0) */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
    if (phase < 0.22) return "Premier Croissant 🌒";
    if (phase < 0.28) return "Premier Quartier 🌓";
    if (phase < 0.47) return "Gibbeuse Croissante 🌔";
    if (phase < 0.53) return "Pleine Lune 🌕";
    if (phase < 0.72) return "Gibbeuse Décroissante 🌖";
    if (phase < 0.78) return "Dernier Quartier 🌗";
    return "Dernier Croissant 🌘";
}

// ===========================================
// FONCTIONS ASTRO UTILITAIRES
// ===========================================

/** Obtient l'heure courante synchronisée (si syncH a réussi) */
function getCDate() {
    if (lLocH === 0) return new Date();
    const currentLocTime = performance.now();
    const offset = currentLocTime - lLocH;
    return new Date(lServH + offset);
}

/** Synchronisation horaire par serveur (UTC/Atomique) */
async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation UTC...';
    const localStartPerformance = performance.now(); 
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;
        lServH = Date.parse(serverData.datetime) + latencyOffset; 
        lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie.`);
    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
/** Calcule l'anomalie solaire moyenne. */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
/** Calcule la longitude écliptique. */
function eclipticLongitude(M) {
    // Calcul de la longitude écliptique du Soleil (L) et des corrections (C et P)
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377; // Longitude du périhélie                                                                
    return M + C + P + Math.PI;
}

/** Calcule le Temps Solaire Vrai (TST) normalisé. */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D', TST_MS: 0 };
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    const epsilon = D2R * (23.4393 - 0.000000356 * d); // Obliquité de l'écliptique
    
    // Calcul de l'Ascension Droite (alpha)
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    
    const meanLongitude = M + D2R * 102.9377 + Math.PI;
    
    // Équation du Temps (EOT = Ascension Droite Moyenne - Ascension Droite Vraie)
    let eot_rad_raw = alpha - meanLongitude; 
    eot_rad_raw = eot_rad_raw % (2 * Math.PI);
    if (eot_rad_raw > Math.PI) { eot_rad_raw -= 2 * Math.PI; } else if (eot_rad_raw < -Math.PI) { eot_rad_raw += 2 * Math.PI; }
    const eot_min = eot_rad_raw * 4 * R2D; // Conversion en minutes (15 deg = 1h, 1 deg = 4 min)
    
    let ecl_long_deg = (L * R2D) % 360; 
    const final_ecl_long = ecl_long_deg < 0 ? ecl_long_deg + 360 : ecl_long_deg;
    
    // Temps Solaire Moyen Local (MST)
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms_raw = msSinceMidnightUTC + mst_offset_ms;
    const mst_ms = (mst_ms_raw % dayMs + dayMs) % dayMs; 
    
    // Temps Solaire Vrai (TST = MST + EOT)
    const eot_ms = eot_min * 60000;
    const tst_ms_raw = mst_ms + eot_ms;
    const tst_ms = (tst_ms_raw % dayMs + dayMs) % dayMs; 
    
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return { TST: toTimeString(tst_ms), TST_MS: tst_ms, MST: toTimeString(mst_ms), EOT: eot_min.toFixed(3), ECL_LONG: final_ecl_long.toFixed(2) };
    }
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js (Logique d'Exécution)
// Dépend de gnss-dashboard-part1.js
// =================================================================

// ... (fonctions précédentes omises)

// ===========================================
// FONCTIONS ASTRO & TEMPS
// ===========================================

/** Met à jour les valeurs Astro, TST et l'Horloge Dynamique sur le DOM */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null || latA === 0 || lonA === 0) return;
    
    // SunCalc fournit les positions et illuminations (influence l'élévation, non la TST pour la rotation)
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    
    // getSolarTime calcule TST en intégrant l'Équation du Temps (EOT)
    const solarTimes = getSolarTime(now, lonA);
    const elevation_deg = sunPos ? (sunPos.altitude * R2D) : 0;
    
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    $('date-display').textContent = now.toLocaleDateString();
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    }
    
    // --- LOGIQUE DE LA CLOCK ET DU CIEL DYNAMIQUE ---
    const TST_hour = solarTimes.TST_MS / 3600000; // Heure TST de 0 à 24
    const clockContainer = $('minecraft-clock');
    
    if (clockContainer) {
        
        // 1. Positionnement des icônes Soleil et Lune
        // Angle TST corrigé : 12h (Midi Solaire) = 0° (Haut/Culmination).
        const TST_normalized = TST_hour % 24; 
        const TST_angle_deg_raw = (TST_normalized - 12) * 15; // 15 degrés par heure
        const TST_angle_deg = (TST_angle_deg_raw + 360) % 360; 
        
        // SUN
        let sunEl = clockContainer.querySelector('.sun-element');
        if (!sunEl) { sunEl = document.createElement('div'); sunEl.className = 'sun-element'; clockContainer.appendChild(sunEl); }
        sunEl.style.transform = `rotate(${TST_angle_deg}deg) translateY(-45px)`; 
        sunEl.textContent = '☀️'; 

        // MOON (Opposé au Soleil, +12h TST)
        let moonEl = clockContainer.querySelector('.moon-element');
        if (!moonEl) { moonEl = document.createElement('div'); moonEl.className = 'moon-element'; clockContainer.appendChild(moonEl); }
        
        const moon_TST_angle_deg = (TST_angle_deg + 180) % 360; // Décalage de 12 heures
        moonEl.style.transform = `rotate(${moon_TST_angle_deg}deg) translateY(-45px)`; 
        
        // --- GESTION DE LA PHASE DE LA LUNE (Visuel par Emoji) ---
        if (moonIllum && moonEl) {
            const phase = moonIllum.phase;
            const phaseEmoji = phase < 0.03 || phase > 0.97 ? '🌑' : 
                               phase < 0.22 ? '🌒' :
                               phase < 0.28 ? '🌓' :
                               phase < 0.47 ? '🌔' :
                               phase < 0.53 ? '🌕' :
                               phase < 0.72 ? '🌖' :
                               phase < 0.78 ? '🌗' : '🌘';
            
            moonEl.textContent = phaseEmoji;
        }

        // 2. Couleur du ciel dynamique (basée sur l'élévation du Soleil)
        let sky_color = '#3f51b5'; // Nuit (Bleu foncé)
        let opacity = 1; 
        
        if (elevation_deg > 10) { 
             sky_color = '#87ceeb'; // Jour (Bleu ciel)
             opacity = 0;
        } else if (elevation_deg > -6) { 
             sky_color = '#ff9800'; // Crépuscule/Aube (Orange/Ambre)
             opacity = 0.5;
        } else if (elevation_deg > -18) { 
             sky_color = '#00008b'; // Crépuscule nautique/astronomique (Bleu nuit)
             opacity = 0.8;
        }

        // Mise à jour du style du pseudo-élément ::before
        const style = document.createElement('style');
        style.innerHTML = `
            #minecraft-clock::before { 
                background-color: ${sky_color} !important; 
                opacity: ${opacity};
            }
        `;
        let oldStyle = clockContainer.querySelector('style');
        if (oldStyle) { clockContainer.removeChild(oldStyle); }
        clockContainer.appendChild(style);

    }
    
    // 3. Mise à jour des valeurs du DOM
    $('time-minecraft').textContent = solarTimes.TST; 
    $('tst').textContent = solarTimes.TST;
    $('lsm').textContent = solarTimes.MST;
    $('sun-elevation').textContent = sunPos ? `${elevation_deg.toFixed(2)} °` : 'N/A';
    $('eot').textContent = solarTimes.EOT + ' min'; 
    $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
    
    // Correction de l'affichage de la durée du jour (SunCalc)
    if (window.SunCalc) {
        const times = SunCalc.getTimes(now, latA, lonA);
        if (times.sunrise && times.sunset) {
            const durationMs = times.sunset.getTime() - times.sunrise.getTime();
            const hours = Math.floor(durationMs / 3600000);
            const minutes = Math.floor((durationMs % 3600000) / 60000);
            $('day-duration').textContent = `${hours}h ${minutes}m`;
        } else {
            $('day-duration').textContent = 'N/A';
        }
    }
}

// ... (Reste des fonctions (updateWeather, handleDeviceMotion, updateDisp, etc.) et de l'initialisation du document, comme dans la version précédente.)
