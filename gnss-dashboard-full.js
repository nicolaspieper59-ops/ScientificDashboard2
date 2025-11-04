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
const LOW_SPEED_THRESHOLD = 1.0; // 👈 NOUVEAU : Seuil de vitesse (m/s) pour amortissement dynamique de R
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

    // 3. NOUVEAU : Boost R aux basses vitesses (non-ZVU) pour fluidifier les mouvements lents.
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

// Les variables globales et fonctions utilitaires sont définies dans part1.js

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
    $('moon-phase-display').textContent = moonIllum ? getMoonPhaseName(moonIllum.phase) : 'N/A';
    
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

// ===========================================
// FONCTIONS API MÉTÉO
// ===========================================

async function updateWeather(latA, lonA) {
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    
    if ($('temp-air')) $('temp-air').textContent = 'Chargement...';

    try {
        const url = `${OWM_API_URL}?lat=${latA.toFixed(4)}&lon=${lonA.toFixed(4)}&units=metric&appid=${OWM_API_KEY}`;
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Weather API failed: ${response.statusText}`);
        const data = await response.json();
        
        const tempC = data.main.temp;
        const pressurehPa = data.main.pressure;
        const humidity = data.main.humidity;
        const windSpeedMs = data.wind.speed; 
        const windDeg = data.wind.deg;
        
        lastP_hPa = pressurehPa; 

        const dewPointC = calculateDewPoint(tempC, humidity);
        // Formule Wind Chill simplifiée (valable < 10°C)
        let windChillC = tempC;
        if (tempC < 10) { 
             windChillC = 13.12 + 0.6215 * tempC - 11.37 * (windSpeedMs**0.16) + 0.3965 * tempC * (windSpeedMs**0.16);
        }

        const directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"];
        const windDirection = directions[Math.round((windDeg / 22.5) + 0.5) % 16];

        $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
        $('pressure').textContent = `${pressurehPa.toFixed(0)} hPa | ${(pressurehPa * 0.75006).toFixed(1)} mmHg`;
        $('humidity').textContent = `${humidity.toFixed(0)} %`;
        $('dew-point').textContent = `${dewPointC.toFixed(1)} °C`;
        $('wind-speed-ms').textContent = `${windSpeedMs.toFixed(2)} m/s | ${(windSpeedMs * 3.6).toFixed(1)} km/h`;
        $('wind-direction').textContent = `${windDirection} (${windDeg} °)`;
        $('temp-feels-like').textContent = `${windChillC.toFixed(1)} °C`;
        $('visibility').textContent = data.visibility ? `${(data.visibility / 1000).toFixed(1)} km` : 'N/A (API)';
        
        // Ajout des données manquantes
        $('uv-index').textContent = data.uvi ? data.uvi.toFixed(1) : 'N/A (API)';
        $('precipitation-rate').textContent = data.rain && data.rain['1h'] ? `${data.rain['1h']} mm/h` : (data.snow && data.snow['1h'] ? `${data.snow['1h']} mm/h (neige)` : '0 mm/h');
        
    } catch (error) {
        console.error("Erreur lors de la récupération des données météo:", error);
        $('temp-air').textContent = 'API ERREUR';
    }
}


// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU)
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // Filtrage passe-bas des accélérations pour la stabilité
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);
    let accel_vertical_lin = 0.0;
    let linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;

    // Détection de la posture statique pour référencer la gravité
    if (Math.abs(kAccel_mag - G_ACC) < ACCEL_MOVEMENT_THRESHOLD) {
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.y = kAccel.y;
        G_STATIC_REF.z = kAccel.z;
        accel_vertical_lin = 0.0; 
    } else {
        // Accélération linéaire (Accélération Totale - Gravité Estimée)
        linear_x = kAccel.x - G_STATIC_REF.x;
        linear_y = kAccel.y - G_STATIC_REF.y;
        linear_z = kAccel.z - G_STATIC_REF.z;
        accel_vertical_lin = linear_z; // Accélération verticale pour l'altitude EKF
    }

    latestVerticalAccelIMU = accel_vertical_lin; 
    // Magnitude de l'accélération linéaire pour le contrôle EKF de la vitesse 3D
    latestLinearAccelMagnitude = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2); 
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
}

// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    if (map) return;
    try {
        map = L.map('map-container').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
        console.log("Carte Leaflet initialisée.");
    } catch (e) {
        console.error("Échec de l'initialisation de la carte Leaflet:", e);
        if ($('map-container')) $('map-container').textContent = 'Erreur d\'initialisation de la carte.';
    }
}

/** Met à jour la carte. */
function updateMap(latA, lonA) {
    if (!map || !marker || !tracePolyline) { initMap(latA, lonA); return; }
    
    const newLatLng = [latA, lonA];
    marker.setLatLng(newLatLng);
    tracePolyline.addLatLng(newLatLng);
    map.setView(newLatLng);
}

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID === null) {
        if ($('freq-select')) $('freq-select').value = currentGPSMode; 
        setGPSMode(currentGPSMode);
        sTime = null; 
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`;
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        $('emergency-stop-btn').classList.remove('active');
    }
    startGPS();
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    const MASS = 70.0; 

    if (now === null) { updateAstro(lat, lon); return; } 
    if (sTime === null) { sTime = now.getTime(); }
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // 1. DÉTERMINATION DU FACTEUR D'AMORTISSEMENT IMU (Dampening)
    let imuDampeningFactor = 1.0; 
    
    if (effectiveAcc <= GOOD_ACC_THRESHOLD) {
        // Très bonne précision (e.g., <= 3m): Réduire l'IMU (Facteur 0.5)
        imuDampeningFactor = 0.5; 
    } else if (effectiveAcc >= MAX_ACC) {
        // Très mauvaise précision (e.g., >= 20m): Pleine confiance à l'IMU (Facteur 1.0)
        imuDampeningFactor = 1.0; 
    } else {
        // Interpolation linéaire entre 0.5 et 1.0
        const range = MAX_ACC - GOOD_ACC_THRESHOLD;
        const value = effectiveAcc - GOOD_ACC_THRESHOLD;
        imuDampeningFactor = 0.5 + 0.5 * (value / range); 
    }

    // 2. Initialisation des Entrées de Contrôle Amorties
    
    // Entrée de contrôle pour le Kalman 3D (Magnitude accélération linéaire amortie)
    let accel_control_3D = latestLinearAccelMagnitude * imuDampeningFactor;
    
    // Entrée de contrôle pour le Kalman Altitude (Accélération verticale amortie)
    let accel_control_V = latestVerticalAccelIMU * imuDampeningFactor;

    // 3. VITESSE 3D INSTANTANÉE (Calculée avant ZVU pour la condition isStaticByDynamicSpeed)
    let spdV_raw = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV_raw = (kAlt - lPos.kAlt_old) / dt; 
    } 
    let spd3D_raw = Math.sqrt(spdH ** 2 + spdV_raw ** 2);

    // 4. LOGIQUE DE MISE À JOUR ZÉRO-VITESSE DYNAMIQUE (ZVU)
    const GPS_NOISE_SPEED = effectiveAcc / VEL_NOISE_FACTOR; 
    const isStaticByIMU = latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD;
    const isStaticByDynamicSpeed = spd3D_raw < GPS_NOISE_SPEED;
    
    const isStatic = isStaticByIMU && isStaticByDynamicSpeed;
    
    let isIMUOnlyMode = false; 
    let spd3D = spd3D_raw;

    if (isStatic) {
        spd3D = 0.0; // Forcer la vitesse 3D brute à 0 pour l'entrée du Kalman 3D
        isIMUOnlyMode = true; 
        
        // CRITIQUE ZVU: Forcer les entrées de contrôle IMU à ZÉRO pour éviter la dérive sur tous les axes.
        accel_control_3D = 0.0; 
        accel_control_V = 0.0; 
        
        // CORRECTION CRITIQUE ZVU: Hard reset de l'état EKF 
        if (kSpd > MIN_SPD) {
            kSpd = 0.0;
            kUncert = 0.01; 
        }
    }
    
    // Détection du Mode Intérieur/Haute Poursuite
    const HIGH_NOISE_ACC_THRESHOLD = 10.0; 
    const isHighNoise = effectiveAcc > HIGH_NOISE_ACC_THRESHOLD || selectedEnvironment === 'CONCRETE' || selectedEnvironment === 'METAL';
    const isInterior = isStatic && isHighNoise;

    // 5. FILTRAGE DE L'ALTITUDE (via Kalman)
    // Utilisation du contrôle vertical amorti/zéroté (accel_control_V)
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, accel_control_V); 
    
    // 6. VITESSE VERTICALE et HORIZONTALE après filtrage d'altitude
    let spdV = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        // Vitesse verticale basée sur le changement d'altitude filtrée (plus stable)
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
        if (isStatic) spdV = 0.0; // S'assurer que spdV est zéro si ZVU est ON
    } 
    
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 
    
    // 7. FILTRE DE KALMAN FINAL (Vitesse 3D Stable)
    let R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa); 
    
    if (isIMUOnlyMode) {
        R_dyn = R_GPS_DISABLED; 
    }

    // Appel à kFilter avec l'entrée de contrôle IMU 3D (accel_control_3D)
    const fSpd = kFilter(spd3D, dt, R_dyn, accel_control_3D); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // Calculs Physiques
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R);
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    // CALCUL DE LA DENSITÉ DE L'AIR
    let airDensity = "N/A";
    const tempElement = $('temp-air').textContent;
    const tempCMatch = tempElement.match(/(-?[\d.]+)\s*°C/);
    
    if (tempCMatch) {
        const tempC = parseFloat(tempCMatch[1]);
        const tempK = tempC + 273.15; 
        const pressurePa = lastP_hPa * 100; 
        const R_specific = 287.058; 

        if (!isNaN(tempK) && tempK > 0 && pressurePa > 0) {
            airDensity = (pressurePa / (R_specific * tempK)).toFixed(3);
        }
    }


De // --- MISE À JOUR DU DOM (GPS/Physique) ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).
                                       } // Fin de la fonction updateDisp (qui contenait le bloc de mise à jour DOM)


// ===========================================
// FONCTIONS API MÉTÉO
// ===========================================

async function updateWeather(latA, lonA) {
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    
    if ($('temp-air')) $('temp-air').textContent = 'Chargement...';

    try {
        const url = `${OWM_API_URL}?lat=${latA.toFixed(4)}&lon=${lonA.toFixed(4)}&units=metric&appid=${OWM_API_KEY}`;
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Weather API failed: ${response.statusText}`);
        const data = await response.json();
        
        const tempC = data.main.temp;
        const pressurehPa = data.main.pressure;
        const humidity = data.main.humidity;
        const windSpeedMs = data.wind.speed; 
        const windDeg = data.wind.deg;
        
        lastP_hPa = pressurehPa; 

        const dewPointC = calculateDewPoint(tempC, humidity);
        // Formule Wind Chill simplifiée (valable < 10°C)
        let windChillC = tempC;
        if (tempC < 10) { 
             windChillC = 13.12 + 0.6215 * tempC - 11.37 * (windSpeedMs**0.16) + 0.3965 * tempC * (windSpeedMs**0.16);
        }

        const directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"];
        const windDirection = directions[Math.round((windDeg / 22.5) + 0.5) % 16];

        $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
        $('pressure').textContent = `${pressurehPa.toFixed(0)} hPa | ${(pressurehPa * 0.75006).toFixed(1)} mmHg`;
        $('humidity').textContent = `${humidity.toFixed(0)} %`;
        $('dew-point').textContent = `${dewPointC.toFixed(1)} °C`;
        $('wind-speed-ms').textContent = `${windSpeedMs.toFixed(2)} m/s | ${(windSpeedMs * 3.6).toFixed(1)} km/h`;
        $('wind-direction').textContent = `${windDirection} (${windDeg} °)`;
        $('temp-feels-like').textContent = `${windChillC.toFixed(1)} °C`;
        $('visibility').textContent = data.visibility ? `${(data.visibility / 1000).toFixed(1)} km` : 'N/A (API)';
        
        // Ajout des données manquantes
        $('uv-index').textContent = data.uvi ? data.uvi.toFixed(1) : 'N/A (API)';
        $('precipitation-rate').textContent = data.rain && data.rain['1h'] ? `${data.rain['1h']} mm/h` : (data.snow && data.snow['1h'] ? `${data.snow['1h']} mm/h (neige)` : '0 mm/h');
        
    } catch (error) {
        console.error("Erreur lors de la récupération des données météo:", error);
        $('temp-air').textContent = 'API ERREUR';
    }
}


// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU)
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // Filtrage passe-bas des accélérations pour la stabilité
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);
    let accel_vertical_lin = 0.0;
    let linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;

    // Détection de la posture statique pour référencer la gravité
    if (Math.abs(kAccel_mag - G_ACC) < ACCEL_MOVEMENT_THRESHOLD) {
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.y = kAccel.y;
        G_STATIC_REF.z = kAccel.z;
        accel_vertical_lin = 0.0; 
    } else {
        // Accélération linéaire (Accélération Totale - Gravité Estimée)
        linear_x = kAccel.x - G_STATIC_REF.x;
        linear_y = kAccel.y - G_STATIC_REF.y;
        linear_z = kAccel.z - G_STATIC_REF.z;
        accel_vertical_lin = linear_z; // Accélération verticale pour l'altitude EKF
    }

    latestVerticalAccelIMU = accel_vertical_lin; 
    // Magnitude de l'accélération linéaire pour le contrôle EKF de la vitesse 3D
    latestLinearAccelMagnitude = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2); 
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
}

// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    if (map) return;
    try {
        map = L.map('map-container').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
        console.log("Carte Leaflet initialisée.");
    } catch (e) {
        console.error("Échec de l'initialisation de la carte Leaflet:", e);
        if ($('map-container')) $('map-container').textContent = 'Erreur d\'initialisation de la carte.';
    }
}

/** Met à jour la carte. */
function updateMap(latA, lonA) {
    if (!map || !marker || !tracePolyline) { initMap(latA, lonA); return; }
    
    const newLatLng = [latA, lonA];
    marker.setLatLng(newLatLng);
    tracePolyline.addLatLng(newLatLng);
    map.setView(newLatLng);
}

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID === null) {
        if ($('freq-select')) $('freq-select').value = currentGPSMode; 
        setGPSMode(currentGPSMode);
        sTime = null; 
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`;
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        $('emergency-stop-btn').classList.remove('active');
    }
    startGPS();
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS ET INTERVALLES
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    if ($('environment-select')) {
        $('environment-select').value = selectedEnvironment;
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
        });
    }

    // Écouteurs pour les boutons et contrôles
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (emergencyStopActive) return;
        wID === null ? startGPS() : stopGPS();
    });
    
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        if (emergencyStopActive) return;
        setGPSMode(e.target.value);
    });
    
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive ? resumeSystem() : emergencyStop(); 
    });
    
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)"; 
    });

    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        distM = 0; timeMoving = 0; 
        if ($('distance-total-km')) $('distance-total-km').textContent = "0.000 km | 0.00 m";
    });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        maxSpd = 0; 
        if ($('speed-max')) $('speed-max').textContent = "0.00000 km/h";
    });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser? (Distance, Max, Kalman)")) { 
            distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; kAlt = 0; kAltUncert = 1000; timeMoving = 0; lastFSpeed = 0;
            if (tracePolyline) tracePolyline.setLatLngs([]); 
        } 
    });
    
    if ($('data-capture-btn')) $('data-capture-btn').addEventListener('click', () => {
        alert("Données actuelles capturées (logique de sauvegarde à implémenter)!");
    });
    
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        $('toggle-mode-btn').textContent = isDarkMode ? "☀️ Mode Jour" : "🌗 Mode Nuit";
    });

    // Démarrage des capteurs IMU
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté ou activé sur cet appareil/navigateur.");
    } 

    syncH(); // Démarrage de la synchronisation de l'heure
    startGPS(); // Démarrage initial du GPS

    // Intervalle lent pour les mises à jour Astro (1s) pour le cas sans signal GPS
    if (domID === null) {
        domID = setInterval(() => {
            // Utilise la dernière position connue
            if (lat !== 0 && lon !== 0) updateAstro(lat, lon); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // Intervalle pour la mise à jour Météo (30s)
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lat !== 0 && lon !== 0) updateWeather(lat, lon);
        }, WEATHER_UPDATE_MS); 
    }
});
