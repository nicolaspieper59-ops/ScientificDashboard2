// =================================================================
// BLOC 1/3 : ekf_logic.js
// Constantes de base, filtres EKF (Vitesse/Altitude) et fonctions de calcul physique/mathématique.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE) ---
const Q_NOISE = 0.1;        // Bruit de processus
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement"
const ALT_TH = -50;         // Seuil d'altitude "Sous-sol"
const MAX_PLAUSIBLE_ACCEL = 20.0; // Anti-spike (m/s²)
const NETHER_RATIO = 8.0;   // Ratio Nether

// --- SEUILS ZUPT (Zero Velocity Update) ---
const ZUPT_RAW_THRESHOLD = 1.0;     // Vitesse brute max (m/s)
const ZUPT_ACCEL_THRESHOLD = 0.5;   // Accélération max (m/s²)

// --- PARAMÈTRES EKF (ALTITUDE) ---
const Q_ALT_NOISE = 0.1;
const R_ALT_MIN = 0.1;

// --- FACTEURS ENVIRONNEMENTAUX (POUR R) ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};

// --- FONCTIONS MATHÉMATIQUES ET PHYSIQUES ---

/** Calcule la distance de Haversine en mètres */
const dist = (lat1, lon1, lat2, lon2, R_ref) => {
    const R = R_ref || R_E_BASE; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
};

/** Calcule l'accélération gravitationnelle locale ou artificielle. */
function getGravityLocal(alt, bodyKey, r_rot, omega_rot) {
    if (bodyKey === 'ROTATING') {
        const centripetal_accel = r_rot * omega_rot ** 2;
        return centripetal_accel; 
    }
    
    if (alt === null) alt = 0;
    const g_base = CELESTIAL_DATA[bodyKey].G;
    const R_base = CELESTIAL_DATA[bodyKey].R;
    
    // Formule de gravité standard
    return g_base * (R_base / (R_base + alt)) ** 2;
}

/** Met à jour les constantes physiques globales lors du changement de corps céleste. */
function updateCelestialBody(bodyKey, alt, r_rot, omega_rot) {
    let G_ACC_local = 0;
    let R_ALT_CENTER_REF_local = R_E_BASE;

    if (bodyKey === 'ROTATING') {
        G_ACC_local = getGravityLocal(alt, bodyKey, r_rot, omega_rot);
        R_ALT_CENTER_REF_local = R_E_BASE;
    } else {
        const data = CELESTIAL_DATA[bodyKey];
        if (data) {
            G_ACC_local = data.G;
            R_ALT_CENTER_REF_local = data.R;
        }
    }
    
    // Retourne les nouvelles valeurs à stocker globalement
    return { G_ACC: G_ACC_local, R_ALT_CENTER_REF: R_ALT_CENTER_REF_local };
}

/** Calcule le Facteur de Rapport de Mouvement (MRF). */
function calculateMRF(alt, netherMode) {
    if (netherMode) {
        return 1.0 / NETHER_RATIO;
    }
    if (alt !== null && alt < ALT_TH) {
        return 0.5; // Facteur arbitraire pour sous-sol
    }
    return 1.0;
}


// --- FONCTIONS DE FILTRAGE (EKF) ---

/**
 * Filtre de Kalman 1D pour la vitesse.
 * @param {number} kSpd_in - Vitesse estimée précédente
 * @param {number} kUncert_in - Incertitude précédente
 * @param {number} nSpd - Nouvelle mesure de vitesse (brute ou 0 si ZUPT)
 * @param {number} dt - Delta temps
 * @param {number} R_dyn - Bruit de mesure (R)
 * @param {number} accel_input - Accélération (de l'IMU, actuellement 0)
 * @returns {object} { kSpd, kUncert } - Nouvel état
 */
function kFilter(kSpd_in, kUncert_in, nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return { kSpd: kSpd_in, kUncert: kUncert_in }; 
    
    const R = R_dyn ?? R_MAX;
    const Q = Q_NOISE * dt * dt; 

    // PRÉDICTION
    let pSpd = kSpd_in + (accel_input * dt); 
    let pUnc = kUncert_in + Q; 

    // CORRECTION
    let K = pUnc / (pUnc + R); 
    let kSpd = pSpd + K * (nSpd - pSpd); 
    let kUncert = (1 - K) * pUnc; 
    
    return { kSpd, kUncert };
}

/** * Applique le filtre de Kalman à l'Altitude. 
 * @param {number} kAlt_in - Altitude estimée précédente
 * @param {number} kAltUncert_in - Incertitude précédente
 * @param {number} nAlt - Nouvelle mesure d'altitude
 * @param {number} acc - Précision de la mesure d'altitude
 * @param {number} dt - Delta temps
 * @returns {object} { kAlt, kAltUncert } - Nouvel état
 */
function kFilterAltitude(kAlt_in, kAltUncert_in, nAlt, acc, dt) {
    if (nAlt === null) return { kAlt: kAlt_in, kAltUncert: kAltUncert_in };
    
    const R_alt = Math.max(R_ALT_MIN, acc * acc); 
    const Q_alt = Q_ALT_NOISE * dt; 
    
    let pAlt = kAlt_in === null ? nAlt : kAlt_in; 
    let pAltUncert = kAlt_in === null ? 1000 : kAltUncert_in + Q_alt;
    
    let K_alt = pAltUncert / (pAltUncert + R_alt);
    let kAlt = pAlt + K_alt * (nAlt - pAlt);
    let kAltUncert = (1 - K_alt) * pAltUncert;
    
    return { kAlt, kAltUncert };
}

/** Calcule le Facteur R (Confiance GPS) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa, selectedEnv) {
    let acc_effective = acc;
    if (acc > MAX_ACC) {
        return 1e9; // Confiance nulle
    }
    
    let R = acc_effective * acc_effective; 
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnv]?.R_MULT || 1.0;
    R *= envFactor;
    
    // Augmente le bruit si la pression atmosphérique (météo) est anormale
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor); 
    }
    
    if (alt !== null && alt < ALT_TH) { 
        R *= 2.0; // Moins de confiance en sous-sol
    } 

    return Math.max(R_MIN, Math.min(R_MAX, R)); 
        }
// =================================================================
// BLOC 2/3 : astro_weather.js
// Logique de services externes : Météo (API), Temps (NTP), et Astro (SunCalc).
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES DE TEMPS & CALENDRIER ---
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms
const J1970 = 2440588, J2000 = 2451545;
const dayMs = 1000 * 60 * 60 * 24;

// --- FONCTIONS DE TEMPS (NTP) ---

/** Synchronise l'horloge interne avec un serveur de temps (UTC/Atomique) */
async function syncH(lServH_in, lLocH_in) {
    let lServH = lServH_in;
    let lLocH = lLocH_in;
    
    if (document.getElementById('local-time')) document.getElementById('local-time').textContent = 'Synchronisation...';
    const localStartPerformance = performance.now(); 

    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        
        const utcTimeISO = serverData.utc_datetime; 
        const serverTimestamp = Date.parse(utcTimeISO); 
        
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;

        lServH = serverTimestamp + latencyOffset; 
        lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie. Latence corrigée: ${latencyOffset.toFixed(1)} ms.`);
        
        const now = getCDate(lServH, lLocH);
        if (now) {
            if (document.getElementById('local-time')) document.getElementById('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if (document.getElementById('date-display')) document.getElementById('date-display').textContent = now.toLocaleDateString('fr-FR');
        }

    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if (document.getElementById('local-time')) document.getElementById('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
    return { lServH, lLocH };
}

/** Retourne l'heure synchronisée (précision RTT compensée en UTC). */
function getCDate(lServH, lLocH) { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// --- FONCTION MÉTÉO ---

/** Récupère et traite les données météo via l'API */
async function fetchWeather(latA, lonA) {
    if (!latA || !lonA) return null; 
    
    const apiUrl = `${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`;
    let weatherData = null;
    
    try {
        const response = await fetch(apiUrl);
        if (!response.ok) throw new Error(`Erreur HTTP: ${response.status}`);
        const data = await response.json();
        
        if (data.main) {
            const tempC = data.main.temp;
            const pressure_hPa = data.main.pressure;
            const humidity_perc = data.main.humidity;
            const tempK = tempC + 273.15;
            
            // Calcul de la densité de l'air
            const pressure_pa = pressure_hPa * 100;
            const air_density = pressure_pa / (R_AIR * tempK);
            
            // Calcul du point de rosée
            const a = 17.27, b = 237.7;
            const h_frac = humidity_perc / 100.0;
            const f = (a * tempC) / (b + tempC) + Math.log(h_frac);
            const dew_point = (b * f) / (a - f);
            
            // Stocke les données pour le DOM et le filtre
            weatherData = {
                tempC: tempC,
                pressure_hPa: pressure_hPa,
                humidity_perc: humidity_perc,
                tempK: tempK,
                air_density: air_density,
                dew_point: dew_point
            };
        } else {
             throw new Error(data.message || 'Données météo incomplètes');
        }
    } catch (err) {
        console.warn("Erreur de récupération météo:", err.message);
    }
    return weatherData; // Retourne les données (ou null en cas d'échec)
}


// --- FONCTIONS ASTRO (SUNCALC) ---

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
/** Calcule l'anomalie solaire moyenne. */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
/** Calcule la longitude écliptique. */
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}
/** Calcule le Temps Solaire Vrai (TST). */
function getSolarTime(date, lon) {
    if (date === null || lon === null || isNaN(lon)) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    
    const J_star = toDays(date) - lon / 360;
    const J_transit = J_star + (0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * L));
    const eot_min = (J_star - J_transit) * 1440; 

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
        NoonSolar: toTimeString(J_transit * dayMs)
    };
}

/** Calcule le temps Minecraft. */
function getMinecraftTime(date) {
    if (date === null) return '00:00:00';
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
    const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;
    };
    return toTimeString(mcTimeMs);
}

/** Retourne le nom de la phase lunaire */
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

/** Met à jour l'horloge visuelle et les couleurs du corps (Day/Night) */
function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const $ = id => document.getElementById(id); // Utilitaire local pour cette fonction
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const clockEl = $('minecraft-clock'); 

    if (!sunEl || !moonEl || !clockEl) return;

    const sunIcon = sunEl.querySelector('.sun-icon');
    const moonIcon = moonEl.querySelector('.moon-icon');

    // 1. Position du Soleil
    if (sunPos) {
        const altDeg = sunPos.altitude * R2D;
        const aziDeg = (sunPos.azimuth * R2D + 180) % 360; 
        sunEl.style.transform = `rotate(${aziDeg}deg)`;
        const radialPercent = Math.min(50, Math.max(0, 50 * (90 - altDeg) / 90));
        const altitudeOffsetPercent = 50 - radialPercent; 
        sunIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercent}%) )`; 
        sunEl.style.display = altDeg > -0.83 ? 'flex' : 'none'; 
    } else {
        sunEl.style.display = 'none';
    }

    // 2. Position de la Lune
    if (moonPos) {
        const altDeg = moonPos.altitude * R2D;
        const aziDeg = (moonPos.azimuth * R2D + 180) % 360; 
        moonEl.style.transform = `rotate(${aziDeg}deg)`;
        const radialPercent = Math.min(50, Math.max(0, 50 * (90 - altDeg) / 90));
        const altitudeOffsetPercent = 50 - radialPercent; 
        moonIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercent}%) )`;
        moonEl.style.display = altDeg > 0 ? 'flex' : 'none';
    } else {
        moonEl.style.display = 'none';
    }

    // 3. Changement de couleur du cadran
    // Les phases sont : Nuit (-0.83°), Aube/Crépuscule (entre -6° et -0.83°), Jour (> -0.83°)
    if (sunPos && sunTimes) {
        const altDeg = sunPos.altitude * R2D;
        let skyClass = 'sky-day';
        let statusText = 'Jour (☀️)';
        
        const nowMs = now.getTime();
        const dawn = sunTimes.dawn.getTime();
        const sunrise = sunTimes.sunrise.getTime();
        const sunset = sunTimes.sunset.getTime();
        const dusk = sunTimes.dusk.getTime();

        if (altDeg < -0.83 && nowMs < dawn) {
            skyClass = 'sky-night'; 
            statusText = 'Nuit (🌙)';
        } else if (altDeg < -0.83 && nowMs > dusk) {
             skyClass = 'sky-night'; 
             statusText = 'Nuit (🌙)';
        } else if (altDeg < -6 || nowMs < sunrise || nowMs > sunset) {
            skyClass = 'sky-twilight'; 
            statusText = 'Crépuscule (✨)';
        }
        
        if (nowMs >= sunset && nowMs < dusk) {
            skyClass = 'sky-sunset'; 
            statusText = 'Coucher de Soleil (🌅)';
        } else if (nowMs >= dawn && nowMs < sunrise) {
            skyClass = 'sky-sunrise';
            statusText = 'Lever de Soleil (🌄)';
        }

        clockEl.className = 'minecraft-clock ' + skyClass;
        $('clock-status').textContent = statusText;
        
    } else if (moonPos && moonPos.altitude * R2D > 0) {
         clockEl.className = 'minecraft-clock sky-night-light';
         $('clock-status').textContent = 'Nuit/Crépuscule (🌙)'; 
    } else {
        $('clock-status').textContent = 'Position Solaire Indisponible';
    }
}

/** Fonction principale de mise à jour Astro (appelée par la boucle lente) */
function updateAstro(latA, lonA, lServH, lLocH) {
    const $ = id => document.getElementById(id); // Utilitaire local 
    const now = getCDate(lServH, lLocH);
    if (now === null) {
        if ($('local-time') && !$('local-time').textContent.includes('Synchronisation')) {
            $('local-time').textContent = 'Synchronisation...';
        }
        return;
    }

    if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);

    if (typeof SunCalc === 'undefined' || !latA || !lonA) {
        $('clock-status').textContent = 'Astro (Attente GPS)...';
        return;
    }

    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    
    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA);
    const sunTimes = SunCalc.getTimes(now, latA, lonA);
    const moonTimes = SunCalc.getMoonTimes(now, latA, lonA, true);
    
    const solarTimes = getSolarTime(now, lonA);

    // Mise à jour du DOM
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('noon-solar')) $('noon-solar').textContent = solarTimes.NoonSolar; // AJOUTÉ : Midi Solaire Local
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min';
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';

    if ($('sun-altitude')) $('sun-altitude').textContent = sunPos.altitude ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('sun-azimuth')) $('sun-azimuth').textContent = sunPos.azimuth ? `${(sunPos.azimuth * R2D).toFixed(2)} °` : 'N/A';
    if ($('moon-alt')) $('moon-alt').textContent = moonPos.altitude ? `${(moonPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('moon-azimuth')) $('moon-azimuth').textContent = moonPos.azimuth ? `${(moonPos.azimuth * R2D).toFixed(2)} °` : 'N/A';
    
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    
    // Mise à jour de la visualisation de l'horloge
    updateClockVisualization(now, sunPos, moonPos, sunTimes);
    
    return { moonIllum, sunTimes, moonTimes };
        }
// =================================================================
// BLOC 3A/3 : main_logic.js (Partie A : Initialisation et Boucles Lentes)
// Variables globales, gestion de l'IMU/Carte, fonctions de contrôle GPS et boucle lente.
// =================================================================

// --- CONSTANTES DE BOUCLE ---
const GPS_HIGH_FREQ_MS = 50;  
const GPS_LOW_FREQ_MS = 200; 
const DOM_SLOW_UPDATE_MS = 1000; // Mise à jour des données non-critiques (1 sec)
let lastMapUpdate = 0;
const MAP_UPDATE_INTERVAL = 3000; // Rafraîchir la vue carte toutes les 3 sec si en mouvement

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000;
let timeMoving = 0;
let lServH = null, lLocH = null;
let lastFSpeed = 0;
let kAlt = null;
let kAltUncert = 10;
let currentGPSMode = 'HIGH_FREQ';
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL';
let currentMass = 70.0;
let R_FACTOR_RATIO = 1.0;
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0;
let gpsAccuracyOverride = 0.0;
let G_ACC = CELESTIAL_DATA['EARTH'].G; // AJOUTÉ : Accélération gravitationnelle locale
let R_ALT_CENTER_REF = R_E_BASE; // AJOUTÉ : Rayon de référence pour les calculs de distance/altitude

// Données externes
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 

// NOUVEAU : Variables IMU
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0; 

// Objets Map
let map, marker, circle;

// --- FONCTION UTILITAIRE DOM ---
const $ = id => document.getElementById(id);

// --- GESTION DES CAPTEURS IMU (AMÉLIORATION) ---
function imuMotionHandler(event) {
    // Utilise l'accélération SANS la gravité si disponible (mieux pour l'EKF)
    const acc = event.accelerationIncludingGravity || event.acceleration;
    if (acc) {
        real_accel_x = acc.x || 0;
        real_accel_y = acc.y || 0;
        real_accel_z = acc.z || 0;
    }
    // TODO: Implémenter la logique de fusion IMU/GPS (pour l'instant, seulement stocké)
}

// --- GESTION DE LA CARTE (LEAFLET) ---
function initMap() {
    try {
        if (typeof L !== 'undefined') {
            map = L.map('map').setView([0, 0], 2);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap contributors' }).addTo(map);
            marker = L.marker([0, 0]).addTo(map);
            circle = L.circle([0, 0], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) {
        console.error("Erreur d'initialisation de Leaflet (Carte):", e);
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte.";
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        circle.setLatLng([lat, lon]).setRadius(acc * R_FACTOR_RATIO);
        
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && kSpd > MIN_SPD) {
            map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16);
            lastMapUpdate = now;
        } else if (map.getZoom() < 10 && (Date.now() - lastMapUpdate > 5000)) {
            map.setView([lat, lon], 12);
            lastMapUpdate = now;
        }
    }
}

// --- FONCTIONS DE CONTRÔLE GPS ---
function setGPSMode(mode) {
    currentGPSMode = mode;
    if (wID !== null) {
        stopGPS(false);
        startGPS();
    }
    if ($('freq-select')) $('freq-select').value = mode;
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = (wID === null) ? `▶️ MARCHE GPS (${mode.replace('_FREQ', '')})` : `⏸️ PAUSE GPS (${mode.replace('_FREQ', '')})`;
    }
}

function startGPS() {
    if (wID !== null) return;
    const intervalMs = currentGPSMode === 'HIGH_FREQ' ? GPS_HIGH_FREQ_MS : GPS_LOW_FREQ_MS;
    const options = { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 };
    wID = navigator.geolocation.watchPosition(updateDisp, gpsError, options);
    
    // Démarre l'écoute IMU si disponible
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', imuMotionHandler, true);
    }
    
    // Démarre la boucle lente si pas encore lancée
    if (domID === null) {
        domID = setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS);
    }
    
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS (${currentGPSMode.replace('_FREQ', '')})`;
    console.log(`GPS Démarré: ${intervalMs} ms (Mode: ${currentGPSMode})`);
}

function stopGPS(resetState = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', imuMotionHandler, true);
    }
    if (resetState) {
        clearInterval(domID);
        domID = null;
    }

    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS (${currentGPSMode.replace('_FREQ', '')})`;
    if ($('speed-status-text')) $('speed-status-text').textContent = "Arrêté.";
    if ($('gps-status')) $('gps-status').textContent = 'GPS Arrêté 🛑';
    console.log("GPS Arrêté.");
}

function gpsError(err) {
    console.error(`Erreur GPS (${err.code}): ${err.message}`);
    if ($('speed-status-text')) $('speed-status-text').textContent = `Erreur GPS: ${err.message}`;
    if ($('gps-status')) $('gps-status').textContent = `Erreur GPS ❌`;
}

// --- BOUCLE LENTE (1 SECONDE) ---
async function slowUpdateLoop() {
    const $ = id => document.getElementById(id); // Utilitaire local

    // --- 1. Synchronisation de l'Heure (NTP) ---
    if (lServH === null || Date.now() - lServH > 600000) { // Resynchronise toutes les 10 minutes
        const syncResult = await syncH(lServH, lLocH);
        lServH = syncResult.lServH;
        lLocH = syncResult.lLocH;
    }

    // --- 2. Mise à jour Météo ---
    if (lat !== null && (lastP_hPa === null || Math.random() < 0.1)) { // Récupère à l'initialisation et aléatoirement
        fetchWeather(lat, lon).then(data => {
            if (data) {
                // Stocke les valeurs pour le filtre EKF
                lastP_hPa = data.pressure_hPa;
                lastT_K = data.tempK;
                lastH_perc = data.humidity_perc / 100.0;
                
                // Met à jour le DOM météo
                if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
                if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                
                // Recalcul de la Vitesse du Son
                // Speed of Sound (m/s) = 331.3 + 0.606 * Temp(C)
                const speedOfSound = 331.3 + 0.606 * data.tempC;
                if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${speedOfSound.toFixed(2)} m/s`;
            }
        });
    }

    // --- 3. Mise à jour Astro ---
    updateAstro(lat, lon, lServH, lLocH);

    // --- 4. Mise à jour de l'Heure Locale (NTP) ---
    const now = getCDate(lServH, lLocH);
    if (now) {
        if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        }
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    }
            }
// ===========================================
// BLOC 3B/3 : main_logic.js (Partie B : Boucle Principale et Initialisation DOM)
// ===========================================
// Exemple de fonction de basculement (dans gnss-dashboard-full.js)
// ...
// Ajouter cette ligne au début de la fonction qui gère le bouton
// L'objectif est de s'assurer que l'état 'isPaused' passe à false.
// ...

if (isPaused) {
    isPaused = false; // Désactiver la pause
    // ...
    // Changer le texte du bouton en "PAUSE GPS"
    // ...
} else {
    isPaused = true; // Activer la pause
    // ...
    // Changer le texte du bouton en "MARCHE GPS"
    // ...
}
// FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp)
function updateDisp(pos) {
    if (emergencyStopActive) return;

    // --- 1. ACQUISITION DES DONNÉES ---
    const cTimePos = pos.timestamp;
    let cLat = pos.coords.latitude;
    let cLon = pos.coords.longitude;
    let altRaw = pos.coords.altitude;
    let accRaw = pos.coords.accuracy;
    let headingRaw = pos.coords.heading;
    
    const now = getCDate(lServH, lLocH);
    if (now === null) { return; }
    if (sTime === null) { sTime = now.getTime(); }
    
    if (gpsAccuracyOverride > 0.0) { accRaw = gpsAccuracyOverride; }

    let isSignalLost = (accRaw > MAX_ACC);
    let modeStatus = '';
    
    // --- 2. GESTION DU BRUIT (R) ET PERTE DE SIGNAL ---
    let R_dyn = getKalmanR(accRaw, kAlt, lastP_hPa, selectedEnvironment);
    const acc = accRaw;

    if (isSignalLost) {
        modeStatus = `⚠️ ESTIMATION SEULE (Signal Perdu/ARRÊTÉ)`;
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Signal Perdu/Estimation)`;
        if (lPos === null) { lPos = pos; return; }
        // En cas de perte de signal, on continue sur la dernière position connue
        cLat = lat;
        cLon = lon;
        altRaw = kAlt;
    } else {
        // Logique de statut de fusion
        if (R_dyn >= R_MAX * 0.75) modeStatus = `🛰️ FUSION FAIBLE (Capteur Domine)`;
        else if (R_dyn <= R_MIN * 1.5) modeStatus = `✅ FUSION HAUTE FIDÉLITÉ (GPS Domine)`;
        else modeStatus = `📶 FUSION EKF (Standard)`;

        if ($('gps-precision')) $('gps-precision').textContent = `✅ ${accRaw.toFixed(0)} m`;
    }
    
    // Temps écoulé
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : 0.05;

    // --- 3. CALCULS DE DISTANCE ET VITESSE BRUTE ---
    let dDist = 0;
    let spd3D_raw = 0; 
    let spd2D_raw = 0;

    if (lPos !== null) {
        // Calcule le MRF (Mouvement Ratio Factor) en fonction du Nether/Sous-sol
        const MRF = calculateMRF(kAlt, netherMode); 
        R_FACTOR_RATIO = 1.0 / MRF;
        
        // La distance est calculée avec le rayon de référence du corps céleste
        dDist = dist(lPos.lat, lPos.lon, cLat, cLon, R_ALT_CENTER_REF) * MRF;
        
        // Distance totale (en mètres)
        distM += dDist;
        
        // Vitesse 2D
        spd2D_raw = dDist / dt; 
        
        // Vitesse 3D (Altitude + 2D)
        const dAlt = kAlt !== null ? (kAlt - (lPos.kAlt_old || kAlt)) : 0; 
        spd3D_raw = Math.sqrt(spd2D_raw ** 2 + (dAlt / dt) ** 2);
    }
    
    // --- 4. FILTRE EKF ALTITUDE ---
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(kAlt, kAltUncert, altRaw, accRaw, dt);
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // --- 5. Lissage anti-spike (accélération plausible) ---
    // Cette étape permet d'éviter les pics de vitesse dus à un mauvais GPS
    if (lPos && lPos.speedMS_3D !== undefined) {
        const lastRawSpd = lPos.speedMS_3D;
        const accelSpike = Math.abs(spd3D_raw - lastRawSpd) / dt;
        if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
            console.warn(`Spike détecté: ${accelSpike.toFixed(2)} m/s². Correction appliquée.`);
            const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
            spd3D_raw = (spd3D_raw > lastRawSpd) ? (lastRawSpd + maxPlausibleChange) : (lastRawSpd - maxPlausibleChange);
        }
    }
    
    // --- 6. LOGIQUE ZUPT (Zero Velocity Update) ---
    let spd_kalman_input = spd3D_raw;
    let R_kalman_input = R_dyn;
    let accel_long_provisional = 0; // Calcul de l'accélération longitudinale
    if (lPos && lPos.speedMS_3D !== undefined && dt > 0.05) {
        accel_long_provisional = (spd3D_raw - lPos.speedMS_3D) / dt;
    }
    
    const isPlausiblyStopped = (
        spd3D_raw < ZUPT_RAW_THRESHOLD &&
        Math.abs(accel_long_provisional) < ZUPT_ACCEL_THRESHOLD &&
        R_dyn < R_MAX
    );

    if (isPlausiblyStopped) {
        spd_kalman_input = 0.0; // Forcer la mesure à 0 m/s
        R_kalman_input = R_MIN; // Confiance maximale dans la mesure ZUPT
        modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)';
    }

    // --- 7. FILTRE EKF VITESSE ---
    // AMÉLIORATION : accel_sensor_input est prêt pour l'IMU, mais à 0 pour l'instant
    let accel_sensor_input = 0; 
    const { kSpd: fSpd, kUncert: fUncert } = kFilter(kSpd, kUncert, spd_kalman_input, dt, R_kalman_input, accel_sensor_input);
    kSpd = fSpd;
    kUncert = fUncert;
    const sSpdFE = kSpd; // Vitesse Stable (EKF)
    
    // --- 8. MISE À JOUR DE L'ÉTAT ET DU STATUT ---
    lat = cLat;
    lon = cLon;
    const isMoving = sSpdFE >= MIN_SPD;
    if (isMoving) {
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, sSpdFE);
    }
    
    // Mise à jour de l'objet de position pour la prochaine itération
    lPos = { 
        timestamp: cTimePos, 
        lat: cLat, 
        lon: cLon, 
        kAlt_old: kAlt, 
        speedMS_3D: spd3D_raw 
    };
    
    if (isSignalLost) {
        if ($('speed-status-text')) $('speed-status-text').textContent = isMoving ? `MOUVEMENT (${modeStatus})` : `ARRÊTÉ (${modeStatus})`;
    } else {
        if ($('speed-status-text')) $('speed-status-text').textContent = isMoving ? `MOUVEMENT EN COURS (${modeStatus})` : `ARRÊTÉ (${modeStatus})`;
    }

    if ($('gps-status')) $('gps-status').textContent = isSignalLost ? `ESTIMATION 🛰️` : `GPS ACTIF 🟢`;

    // --- 9. MISE À JOUR DU DOM (VITESSE & DISTANCE) ---
    
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(2)} km/h`; // UTILISE toFixed(2) pour le gros affichage
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = `${(sSpdFE / 1000).toExponential(3)} km/s`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? `${(distM / timeMoving * KMH_MS).toFixed(5)} km/h` : '0.00000 km/h';
    if ($('speed-avg-total')) $('speed-avg-total').textContent = (now.getTime() - sTime) / 1000 > 1 ? `${(distM / ((now.getTime() - sTime) / 1000) * KMH_MS).toFixed(5)} km/h` : '0.00000 km/h';
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd3D_raw.toFixed(5)} m/s`;
    
    // Calculs Physiques
    const mach = sSpdFE / (331.3 + 0.606 * (lastT_K ? lastT_K - 273.15 : 15)); // Approximation à 15°C si pas de données
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / C_S * 100).toFixed(2)} %`;
    if ($('mach-number')) $('mach-number').textContent = `${mach.toFixed(4)}`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)}%`;
    if ($('gravity-accel')) $('gravity-accel').textContent = `${getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity).toFixed(4)} m/s²`;
    if ($('dynamic-mass')) $('dynamic-mass').textContent = lastH_perc !== null ? `${(currentMass / (1.0 - lastH_perc)).toFixed(3)} kg` : `${currentMass.toFixed(3)} kg`;
    
    // Distance
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('distance-light-s')) $('distance-light-s').textContent = `${(distM / C_L).toExponential(2)} s`;
    if ($('distance-light-min')) $('distance-light-min').textContent = `${(distM / C_L / 60).toExponential(2)} min`;
    if ($('distance-light-h')) $('distance-light-h').textContent = `${(distM / C_L / 3600).toExponential(2)} h`;
    if ($('distance-light-day')) $('distance-light-day').textContent = `${(distM / C_L / 86400).toExponential(2)} jours`;

    // Temps
    if ($('elapsed-time')) $('elapsed-time').textContent = `${((now.getTime() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    
    // Altitude
    if ($('altitude-raw')) $('altitude-raw').textContent = altRaw === null ? 'N/A' : `${altRaw.toFixed(2)} m`;
    if ($('altitude-ekf')) $('altitude-ekf').textContent = kAlt === null ? 'N/A' : `${kAlt.toFixed(2)} m`;
    if ($('alt-uncert-ekf')) $('alt-uncert-ekf').textContent = kAltUncert.toFixed(2) + ' m';
    
    // Position
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(5)} °`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(5)} °`;
    if ($('heading-raw')) $('heading-raw').textContent = headingRaw === null ? 'N/A' : `${headingRaw.toFixed(1)} °`;
    
    // EKF
    if ($('ekf-uncert')) $('ekf-uncert').textContent = `${kUncert.toFixed(2)} m/s²`;
    if ($('ekf-R')) $('ekf-R').textContent = R_dyn.toFixed(2);
    if ($('ekf-env-factor')) $('ekf-env-factor').textContent = ENVIRONMENT_FACTORS[selectedEnvironment]?.DISPLAY || 'Normal';
    
    // Mise à jour de la carte
    updateMap(lat, lon, accRaw)
}

// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================
document.addEventListener('DOMContentLoaded', () => {
    initMap(); // Initialise la carte

    // --- Initialisation des Contrôles ---
    const massInput = $('mass-input');
    if (massInput) {
        massInput.addEventListener('input', () => {
            currentMass = parseFloat(massInput.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        currentMass = parseFloat(massInput.value);
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    }
    
    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => {
            // Utilise les fonctions de ekf_logic.js
            const newVals = updateCelestialBody(e.target.value, kAlt, rotationRadius, angularVelocity);
            G_ACC = newVals.G_ACC;
            R_ALT_CENTER_REF = newVals.R_ALT_CENTER_REF; // AJOUTÉ : Mise à jour du rayon de référence
            currentCelestialBody = e.target.value;
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        });
    }

    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius')?.value) || 100;
        angularVelocity = parseFloat($('angular-velocity')?.value) || 0;
        
        if ($('rotation-radius-display')) $('rotation-radius-display').textContent = `${rotationRadius.toFixed(0)} m`;
        if ($('angular-velocity-display')) $('angular-velocity-display').textContent = `${angularVelocity.toFixed(4)} rad/s`;

        if (currentCelestialBody === 'ROTATING') {
            // Utilise les fonctions de ekf_logic.js
            const newVals = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            G_ACC = newVals.G_ACC;
            R_ALT_CENTER_REF = newVals.R_ALT_CENTER_REF; // AJOUTÉ : Mise à jour du rayon de référence
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        }
    };
    
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);
    updateRotation(); // Initialisation

    if ($('nether-mode-toggle')) {
         $('nether-mode-toggle').addEventListener('change', (e) => {
            netherMode = e.target.checked;
            e.target.parentNode.querySelector('span').textContent = netherMode ? 'ACTIVÉ' : 'DÉSACTIVÉ';
        });
    }
    
    if ($('env-select')) {
        $('env-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
        });
    }

    if ($('gps-acc-override')) {
        $('gps-acc-override').addEventListener('input', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
            if (gpsAccuracyOverride < 0) gpsAccuracyOverride = 0;
            e.target.parentNode.querySelector('span').textContent = `${gpsAccuracyOverride.toFixed(1)} m`;
        });
        $('gps-acc-override').value = gpsAccuracyOverride;
        $('gps-acc-override').parentNode.querySelector('span').textContent = `${gpsAccuracyOverride.toFixed(1)} m`;
    }

    if ($('xray-button')) {
        $('xray-button').addEventListener('click', () => {
            $('minecraft-clock').classList.toggle('x-ray');
        });
    }

    // Boutons de contrôle
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => {
            (wID === null) ? startGPS() : stopGPS();
        });
    }
    
    if ($('freq-select')) {
        $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    }
    
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').addEventListener('click', () => {
            emergencyStopActive = !emergencyStopActive;
            const btn = $('emergency-stop-btn');
            btn.classList.toggle('active', emergencyStopActive);
            btn.textContent = emergencyStopActive ? '🛑 Arrêt d\'urgence: ACTIF' : '🛑 Arrêt d\'urgence: INACTIF 🟢';
            if (emergencyStopActive) stopGPS(false);
        });
    }
    
    if ($('reset-dist-btn')) {
        $('reset-dist-btn').addEventListener('click', () => {
            distM = 0;
            timeMoving = 0;
            if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`;
        });
    }

    if ($('reset-max-btn')) {
        $('reset-max-btn').addEventListener('click', () => {
            maxSpd = 0;
            if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`;
        });
    }

    if ($('reset-all-btn')) {
        $('reset-all-btn').addEventListener('click', () => {
            // Réinitialisation douce des valeurs d'état
            distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; timeMoving = 0; lPos = null; sTime = null;
            if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`;
            if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`;
        });
    }
    
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
        });
    }

    // --- DÉMARRAGE DU SYSTÈME ---
    // Initialise les valeurs de gravité (utilise ekf_logic.js)
    const initVals = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    G_ACC = initVals.G_ACC;
    R_ALT_CENTER_REF = initVals.R_ALT_CENTER_REF; // AJOUTÉ : Initialisation correcte
    if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    
    // Démarre la boucle lente immédiatement
    slowUpdateLoop();
});
