// =================================================================
// BLOC 1/3 : ekf_logic.js
// Constantes de base, filtres EKF (Vitesse/Altitude) et fonctions de calcul physique/math茅matique.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATH脡MATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumi猫re (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante sp茅cifique de l'air sec (J/kg路K)

// --- PARAM脠TRES DU FILTRE DE KALMAN (VITESSE) ---
const Q_NOISE = 0.1;        // Bruit de processus
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;        // Pr茅cision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement"
const ALT_TH = -50;         // Seuil d'altitude "Sous-sol"
const MAX_PLAUSIBLE_ACCEL = 20.0; // Anti-spike (m/s虏)
const NETHER_RATIO = 8.0;   // Ratio Nether

// --- SEUILS ZUPT (Zero Velocity Update) ---
const ZUPT_RAW_THRESHOLD = 1.0;     // Vitesse brute max (m/s)
const ZUPT_ACCEL_THRESHOLD = 0.5;   // Acc茅l茅ration max (m/s虏)

// --- PARAM脠TRES EKF (ALTITUDE) ---
const Q_ALT_NOISE = 0.1;
const R_ALT_MIN = 0.1;

// --- FACTEURS ENVIRONNEMENTAUX (POUR R) ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'For锚t' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'M茅tal/B芒timent' },
};

// --- DONN脡ES C脡LESTES/GRAVIT脡 ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};

// --- FONCTIONS MATH脡MATIQUES ET PHYSIQUES ---

/** Calcule la distance de Haversine en m猫tres */
const dist = (lat1, lon1, lat2, lon2, R_ref) => {
    const R = R_ref || R_E_BASE; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
};

/** Calcule l'acc茅l茅ration gravitationnelle locale ou artificielle. */
function getGravityLocal(alt, bodyKey, r_rot, omega_rot) {
    if (bodyKey === 'ROTATING') {
        const centripetal_accel = r_rot * omega_rot ** 2;
        return centripetal_accel; 
    }
    
    if (alt === null) alt = 0;
    const g_base = CELESTIAL_DATA[bodyKey].G;
    const R_base = CELESTIAL_DATA[bodyKey].R;
    
    // Formule de gravit茅 standard
    return g_base * (R_base / (R_base + alt)) ** 2;
}

/** Met 脿 jour les constantes physiques globales lors du changement de corps c茅leste. */
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
    
    // Retourne les nouvelles valeurs 脿 stocker globalement
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
 * @param {number} kSpd_in - Vitesse estim茅e pr茅c茅dente
 * @param {number} kUncert_in - Incertitude pr茅c茅dente
 * @param {number} nSpd - Nouvelle mesure de vitesse (brute ou 0 si ZUPT)
 * @param {number} dt - Delta temps
 * @param {number} R_dyn - Bruit de mesure (R)
 * @param {number} accel_input - Acc茅l茅ration (de l'IMU, actuellement 0)
 * @returns {object} { kSpd, kUncert } - Nouvel 茅tat
 */
function kFilter(kSpd_in, kUncert_in, nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return { kSpd: kSpd_in, kUncert: kUncert_in }; 
    
    const R = R_dyn ?? R_MAX;
    const Q = Q_NOISE * dt * dt; 

    // PR脡DICTION
    let pSpd = kSpd_in + (accel_input * dt); 
    let pUnc = kUncert_in + Q; 

    // CORRECTION
    let K = pUnc / (pUnc + R); 
    let kSpd = pSpd + K * (nSpd - pSpd); 
    let kUncert = (1 - K) * pUnc; 
    
    return { kSpd, kUncert };
}

/** * Applique le filtre de Kalman 脿 l'Altitude. 
 * @param {number} kAlt_in - Altitude estim茅e pr茅c茅dente
 * @param {number} kAltUncert_in - Incertitude pr茅c茅dente
 * @param {number} nAlt - Nouvelle mesure d'altitude
 * @param {number} acc - Pr茅cision de la mesure d'altitude
 * @param {number} dt - Delta temps
 * @returns {object} { kAlt, kAltUncert } - Nouvel 茅tat
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
    
    // Augmente le bruit si la pression atmosph茅rique (m茅t茅o) est anormale
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
// Logique de services externes : M茅t茅o (API), Temps (NTP), et Astro (SunCalc).
// =================================================================

// --- CL脡S D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES DE TEMPS & CALENDRIER ---
const MC_DAY_MS = 72 * 60 * 1000; // Dur茅e d'un jour Minecraft en ms
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
        console.log(`Synchronisation UTC Atomique r茅ussie. Latence corrig茅e: ${latencyOffset.toFixed(1)} ms.`);
        
        const now = getCDate(lServH, lLocH);
        if (now) {
            if (document.getElementById('local-time')) document.getElementById('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if (document.getElementById('date-display')) document.getElementById('date-display').textContent = now.toLocaleDateString('fr-FR');
        }

    } catch (error) {
        console.warn("脡chec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if (document.getElementById('local-time')) document.getElementById('local-time').textContent = 'N/A (SYNCHRO 脡CHOU脡E)';
    }
    return { lServH, lLocH };
}

/** Retourne l'heure synchronis茅e (pr茅cision RTT compens茅e en UTC). */
function getCDate(lServH, lLocH) { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// --- FONCTION M脡T脡O ---

/** R茅cup猫re et traite les donn茅es m茅t茅o via l'API */
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
            
            // Calcul de la densit茅 de l'air
            const pressure_pa = pressure_hPa * 100;
            const air_density = pressure_pa / (R_AIR * tempK);
            
            // Calcul du point de ros茅e
            const a = 17.27, b = 237.7;
            const h_frac = humidity_perc / 100.0;
            const f = (a * tempC) / (b + tempC) + Math.log(h_frac);
            const dew_point = (b * f) / (a - f);
            
            // Stocke les donn茅es pour le DOM et le filtre
            weatherData = {
                tempC: tempC,
                pressure_hPa: pressure_hPa,
                humidity_perc: humidity_perc,
                tempK: tempK,
                air_density: air_density,
                dew_point: dew_point
            };
        } else {
             throw new Error(data.message || 'Donn茅es m茅t茅o incompl猫tes');
        }
    } catch (err) {
        console.warn("Erreur de r茅cup茅ration m茅t茅o:", err.message);
    }
    return weatherData; // Retourne les donn茅es (ou null en cas d'茅chec)
}


// --- FONCTIONS ASTRO (SUNCALC) ---

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
/** Calcule l'anomalie solaire moyenne. */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
/** Calcule la longitude 茅cliptique. */
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
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 馃寫";
    if (phase < 0.23) return "Premier Croissant 馃寬";
    if (phase < 0.27) return "Premier Quartier 馃寭";
    if (phase < 0.48) return "Gibbeuse Croissante 馃寯";
    if (phase < 0.52) return "Pleine Lune 馃寱";
    if (phase < 0.73) return "Gibbeuse D茅croissante 馃寲";
    if (phase < 0.77) return "Dernier Quartier 馃寳";
    return "Dernier Croissant 馃寴"; 
}

/** Met 脿 jour l'horloge visuelle et les couleurs du corps (Day/Night) */
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
    
    // 3. Fond du corps et de l'horloge
    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'sky-night-light', 'dark-mode', 'light-mode');
    clockEl.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'sky-night-light');

    if (sunTimes) {
        const nowMs = now.getTime();
        let bodyClass;
        if (nowMs >= sunTimes.sunriseEnd.getTime() && nowMs < sunTimes.sunsetStart.getTime()) {
            bodyClass = 'sky-day';
        } else if (nowMs >= sunTimes.dusk.getTime() || nowMs < sunTimes.dawn.getTime()) {
            bodyClass = 'sky-night';
        } else {
            bodyClass = 'sky-sunset';
        }
        
        body.classList.add(bodyClass);
        body.classList.add(bodyClass === 'sky-day' ? 'light-mode' : 'dark-mode');
        clockEl.classList.add(bodyClass); 

        $('clock-status').textContent = sunPos && sunPos.altitude > 0 ? 'Jour Solaire (鈽�锔�)' : 'Nuit/Cr茅puscule (馃寵)';
    } else {
        $('clock-status').textContent = 'Position Solaire Indisponible';
    }
}

/** Fonction principale de mise 脿 jour Astro (appel茅e par la boucle lente) */
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
    
    // Mise 脿 jour du DOM
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' 掳';

    if ($('sun-altitude')) $('sun-altitude').textContent = `${(sunPos.altitude * R2D).toFixed(2)} 掳`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} 掳 (S-O)`;
    
    if ($('moon-altitude')) $('moon-altitude').textContent = `${(moonPos.altitude * R2D).toFixed(2)} 掳`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)} 掳 (S-O)`;
    
    if ($('moon-illum-fraction')) $('moon-illum-fraction').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';
    if ($('day-duration') && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const hours = Math.floor(durationMs / 3600000);
        const minutes = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${hours}h ${minutes}m`;
    } else if ($('day-duration')) {
        $('day-duration').textContent = 'N/A (Polaire/Nuit)';
    }

    if ($('moon-times')) $('moon-times').textContent = moonTimes ? 
        `鈫� ${moonTimes.rise ? moonTimes.rise.toLocaleTimeString() : 'N/A'} / 鈫� ${moonTimes.set ? moonTimes.set.toLocaleTimeString() : 'N/A'}` : 'N/D';

    updateClockVisualization(now, sunPos, moonPos, sunTimes);
                }
// ======================================================
