// =================================================================
// BLOC 1/4 : Constantes Universelles, EKF Cœur et Mathématiques
// Fusion de ekf_logic.js (modifié) et des constantes fondamentales
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const C_S_STD = 343;            // Vitesse du son standard (m/s)
const G_U = 6.67430e-11;        // Constante gravitationnelle universelle (N·m²/kg²)
const H_PLANCK = 6.62607015e-34; // Constante de Planck (J·s)
const K_BOLTZMANN = 1.380649e-23; // Constante de Boltzmann (J/K)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air
const MU_DYNAMIC_AIR = 1.8e-5;  // Viscosité dynamique de l'air (Pa·s)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const C_DRAG_AREA = 0.5;        // CdA: Coefficient de traînée * Surface (valeur par défaut)
const SAT_O2_SEA_LEVEL = 97.5;  // Saturation O2 théorique au niveau de la mer (%)

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


// --- FONCTIONS MATHÉMATIQUES ET PHYSIQUES CORE ---

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
            G_ACC_local = getGravityLocal(alt, bodyKey, r_rot, omega_rot); // Recalculer pour l'altitude
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


// --- FONCTIONS DE FILTRAGE (EKF / Kalman 1D) ---

/**
 * Filtre de Kalman 1D pour la vitesse.
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

/** * Applique le filtre de Kalman à l'Altitude. */
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
// BLOC 2/4 : Modèles de Physique Avancée, Relativité et Géodésie
// Fonctions de calcul des grandeurs scientifiques de pointe.
// =================================================================

/**
 * Calcule les grandeurs de Physique Avancée, Relativité et Fluides.
 */
function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, tempK, airDensity, lat, kAltUncert) {
    const V = kSpd;
    const V_kmh = V * KMH_MS;
    const alt = kAlt;

    // --- 1. Relativité et Énergie
    const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(V / C_L, 2));
    const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
    const E0 = mass * C_L * C_L;
    const energyRelativistic = lorentzFactor * E0;
    const momentum = mass * V; // Quantité de mouvement

    // Correction de la Dilatation Temporelle Gravitationnelle (Relativité Générale)
    // Approximation: 1 ns/jour/m d'altitude
    const DILATION_FACTOR = 1e-18; 
    const gravitationalDilation = alt * DILATION_FACTOR * 86400 * 1e9; // ns/jour

    // Rayon de Schwarzschild (pour la masse de l'objet)
    const Rs_object = (2 * G_U * mass) / (C_L * C_L);
    
    // --- 2. Mécanique des Fluides
    const speedOfSoundLocal = Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK); // C_s = sqrt(γRT)
    const machNumber = V / speedOfSoundLocal;
    const dynamicPressure = 0.5 * airDensity * V * V; // q = 1/2 ρV²
    const viscosityCinematic = MU_DYNAMIC_AIR / airDensity;
    const reynoldsNumber = (airDensity * V * 1) / MU_DYNAMIC_AIR; // Longueur caractéristique L=1m pour la simu
    const dragForce = dynamicPressure * CdA; // F_D = q * CdA

    // --- 3. Géophysique
    // Force de Coriolis (simplifiée)
    const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
    
    // Altitude Géopotentielle (simplifiée par la gravité locale)
    const G_ACC_ref = 9.80665;
    const geopotentialAltitude = alt * (G_ACC_ref / (G_ACC_ref + OMEGA_EARTH * OMEGA_EARTH * R_E_BASE * Math.cos(lat * D2R) ** 2));

    // --- 4. EKF / Métrologie Avancée
    // Bande Passante (Fréquence de Coupure, f_N = 1/2 * f_s)
    const EKF_UPDATE_RATE = 1000; // Fréquence de la boucle DOM en ms (du Bloc 4)
    const nyquistFrequency = 0.5 * (1000 / EKF_UPDATE_RATE); // Hz (très basse ici)

    // Ellipse d'Erreur 1D (Incertitude d'Altitude)
    const altSigma = Math.sqrt(kAltUncert);

    return { 
        lorentzFactor, timeDilationSpeed, energyRelativistic, E0, momentum, gravitationalDilation, Rs_object,
        speedOfSoundLocal, machNumber, dynamicPressure, reynoldsNumber, viscosityCinematic, dragForce,
        coriolisForce, geopotentialAltitude,
        nyquistFrequency, altSigma
    };
}

/**
 * Calcule les grandeurs de Bio-Physique et de Stress.
 */
function calculateBioSVT(tempC, alt, humidity_perc, pressurePa, sunAltitudeRad) {
    // 1. Point de Rosée (déjà calculé dans fetchWeather, mais calculé ici pour l'isolation)
    const a = 17.27, b = 237.7;
    const h_frac = humidity_perc / 100.0;
    const f = (a * tempC) / (b + tempC) + Math.log(h_frac);
    const dewPoint = (b * f) / (a - f);

    // 2. Température du Thermomètre Mouillé (Tw) (Psychrométrie)
    // Formule simplifiée de Stull
    const wetBulbTemp = tempC * Math.atan(0.151977 * Math.sqrt(h_frac + 8.313659)) + 
                        Math.atan(tempC + h_frac) - 
                        Math.atan(h_frac - 1.67633) + 
                        0.00391838 * Math.pow(h_frac, 1.5) * Math.atan(0.023101 * h_frac) - 4.686035;

    // 3. CAPE (Convective Available Potential Energy) - Simplification
    const CAPE_sim = Math.max(0, 0.5 * (tempC - dewPoint) * 500); // 0.5 * g * ΔT * ΔZ (simplifié)

    // 4. Saturation en Oxygène (SVT) - Modèle d'altitude
    const O2Saturation = SAT_O2_SEA_LEVEL - (alt / 1000) * 2; // Perte de 2% par km (simplifié)
    const O2SaturationClamped = Math.max(70, Math.min(100, O2Saturation));

    // 5. Taux de Photosynthèse (SVT) - Modèle simplifié par température et lumière (angle du soleil)
    const sunAngleFactor = Math.max(0, sunAltitudeRad * R2D) / 90; // 0 à 1
    const tempFactor = Math.exp(-0.5 * Math.pow((tempC - 25) / 10, 2)); // Pic à 25°C
    const photosynthesisRate = 0.05 * sunAngleFactor * tempFactor; // mol CO2 / m² / s
    
    // 6. Taux de Respiration Cellulaire (SVT) - Modèle d'Arrhenius (simplifié)
    const respirationRate = 0.001 * Math.exp(0.06 * tempC); // Augmentation exponentielle avec la température

    // 7. Rayonnement Solaire (Irradiance)
    const solarIrradiance = 1000 * Math.max(0, Math.sin(sunAltitudeRad)); // 1000 W/m² max
    const radiationPressure = solarIrradiance / C_L; // Pression de radiation

    return { 
        dewPoint, wetBulbTemp, CAPE_sim, O2SaturationClamped, photosynthesisRate, respirationRate, 
        solarIrradiance, radiationPressure
    };
        }
// =================================================================
// BLOC 3/4 : Services Externes, Météo, Astro, et Biophysique Core
// Logic de services externes : Météo (API), Temps (NTP), et Astro (SunCalc).
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
        
        const now = getCDate(lServH, lLocH);
        if (now) {
            if (document.getElementById('local-time')) document.getElementById('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if (document.getElementById('date-display')) document.getElementById('date-display').textContent = now.toLocaleDateString('fr-FR');
        }

    } catch (error) {
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
async function fetchWeather(latA, lonA, kAlt) {
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
            const air_density = pressure_pa / (R_SPECIFIC_AIR * tempK);
            
            // Calcul de l'Humidité Absolue
            const P_sat_hPa = tC => 6.1078 * Math.pow(10, (7.5 * tC) / (237.3 + tC));
            const P_v = P_sat_hPa(tempC) * (humidity_perc / 100.0); // Pression vapeur réelle (hPa)
            const absoluteHumidity = (P_v * 100 * 18.015) / (8.314 * tempK); // (Pa * M_eau) / (R * T)

            // Calcul du point de rosée (déjà dans Bloc 2/4 mais stocké ici)
            const a = 17.27, b = 237.7;
            const h_frac = humidity_perc / 100.0;
            const f = (a * tempC) / (b + tempC) + Math.log(h_frac);
            const dew_point = (b * f) / (a - f);
            
            // Stocke les données
            weatherData = {
                tempC: tempC,
                pressure_hPa: pressure_hPa,
                humidity_perc: humidity_perc,
                tempK: tempK,
                air_density: air_density,
                dew_point: dew_point,
                absoluteHumidity: absoluteHumidity
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

    // ... (Logique de mise à jour de l'affichage du soleil/lune et du ciel, omise pour la concision) ...

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

        $('clock-status').textContent = sunPos && sunPos.altitude > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';
    } else {
        $('clock-status').textContent = 'Position Solaire Indisponible';
    }
}

/** Fonction principale de mise à jour Astro (appelée par la boucle lente) */
function updateAstro(latA, lonA, lServH, lLocH) {
    const $ = id => document.getElementById(id); // Utilitaire local
    const now = getCDate(lServH, lLocH); 
    
    if (now === null) { return; }
    
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
    
    // Stocke la position du soleil en radians pour les calculs biophysiques
    let sunAltitudeRad = sunPos.altitude;
    
    // Mise à jour du DOM
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    // ... (Mise à jour des autres champs astro) ...

    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    
    updateClockVisualization(now, sunPos, moonPos, sunTimes);

    return { sunAltitudeRad };
}
// =================================================================
// BLOC 4/4 : Logique Applicative, Gestion des Capteurs et DOM
// État global, gestion des capteurs (GPS/IMU), boucle de mise à jour (updateDisp),
// et initialisation DOM.
// =================================================================

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 1000; // Rafraîchissement des données non-critiques (1 sec)
let lastMapUpdate = 0; 
const MAP_UPDATE_INTERVAL = 3000; 

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
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
let currentCdA = C_DRAG_AREA; // Utilise la constante de BLOC 1
let R_FACTOR_RATIO = 1.0;
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let gpsAccuracyOverride = 0.0; 
let G_ACC = CELESTIAL_DATA['EARTH'].G;
let R_ALT_CENTER_REF = R_E_BASE;

// Données externes et calculées pour l'affichage avancé
let lastWeatherData = null; 
let sunAltitudeRad = 0;

// NOUVEAU : Variables IMU
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;

// Objets Map
let map, marker, circle;

// --- FONCTION UTILITAIRE DOM ---
const $ = id => document.getElementById(id);


// --- GESTION DES CAPTEURS IMU (AMÉLIORATION) ---
function imuMotionHandler(event) {
    if (event.acceleration) {
        real_accel_x = event.acceleration.x || 0;
        real_accel_y = event.acceleration.y || 0;
        real_accel_z = event.acceleration.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Sans Gravité)";
    } 
    else if (event.accelerationIncludingGravity) {
        real_accel_x = event.accelerationIncludingGravity.x || 0;
        real_accel_y = event.accelerationIncludingGravity.y || 0;
        real_accel_z = event.accelerationIncludingGravity.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Avec Gravité)";
    } else {
        if ($('imu-status')) $('imu-status').textContent = "Erreur (Capteur N/A)";
    }
}

function startIMUListeners() {
    // ... (Logique de gestion des permissions IMU, inchangée) ...
}

function stopIMUListeners() {
    // ... (Logique d'arrêt des listeners, inchangée) ...
}


// --- Fonctions Carte ---
function initMap() {
    // ... (Logique d'initialisation de la carte Leaflet, inchangée) ...
}

function updateMap(lat, lon, acc) {
    // ... (Logique de mise à jour de la carte, inchangée) ...
}


// --- FONCTIONS DE CONTRÔLE GPS ---

function setGPSMode(mode) {
    // ... (Logique de changement de mode GPS, inchangée) ...
}

function startGPS() {
    // ... (Logique de démarrage GPS, inchangée) ...
}

function stopGPS(resetButton = true) {
    // ... (Logique d'arrêt GPS, inchangée) ...
}

function emergencyStop() {
    // ... (Logique d'arrêt d'urgence, inchangée) ...
}

function resumeSystem() {
    // ... (Logique de reprise du système, inchangée) ...
}

function handleErr(err) {
    // ... (Logique de gestion des erreurs GPS, inchangée) ...
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    // --- 1. ACQUISITION DES DONNÉES & FILTRES ---
    // ... (Logique de récupération GPS, gestion du temps, EKF Altitude, Anti-Spike, ZUPT) ...
    
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
    let R_dyn = getKalmanR(accRaw, kAlt, lastWeatherData ? lastWeatherData.pressure_hPa : null, selectedEnvironment); 
    const acc = accRaw; 

    if (isSignalLost) { 
        // ... (Gestion de la perte de signal) ...
        cLat = lat; cLon = lon; altRaw = kAlt; 
    } else {
        lat = cLat; lon = cLon;
    }
    
    let dt = 0;
    if (lPos) { dt = (cTimePos - lPos.timestamp) / 1000; } else { /* Initialisation */ lPos = pos; lPos.speedMS_3D = 0; lPos.kAlt_old = altRaw; kAlt = altRaw; updateMap(cLat, cLon, accRaw); return; }
    if (dt < MIN_DT || dt > 10) { lPos = pos; return; }

    // EKF ALTITUDE
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(kAlt, kAltUncert, altRaw, pos.coords.altitudeAccuracy || R_ALT_MIN, dt);
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // ZUPT / FILTRE EKF VITESSE
    // ... (Calcul de spd3D_raw, anti-spike, logique ZUPT) ...
    const { kSpd: fSpd, kUncert: kUncert_new } = kFilter(kSpd, kUncert, spd_kalman_input, dt, R_kalman_input, accel_sensor_input);
    kSpd = fSpd;
    kUncert = kUncert_new;
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    
    // --- 2. CALCULS AVANCÉS ---
    
    let accel_long = 0;
    if (dt > 0.05) { accel_long = (sSpdFE - lastFSpeed) / dt; }
    lastFSpeed = sSpdFE;

    R_FACTOR_RATIO = calculateMRF(kAlt_new, netherMode); 
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const local_g = getGravityLocal(kAlt_new, currentCelestialBody, rotationRadius, angularVelocity); 
    
    // NOUVEAU : Calculs de Physique et Biophysique Avancée
    const advancedPhysics = calculateAdvancedPhysics(sSpdFE, kAlt_new, currentMass, currentCdA, 
        lastWeatherData ? lastWeatherData.tempK : 293.15, 
        lastWeatherData ? lastWeatherData.air_density : 1.225, 
        cLat, kAltUncert_new);
        
    const bioSVT = calculateBioSVT(
        lastWeatherData ? lastWeatherData.tempC : 20.0, 
        kAlt_new, 
        lastWeatherData ? lastWeatherData.humidity_perc : 50.0, 
        lastWeatherData ? lastWeatherData.pressure_hPa * 100 : 101325, 
        sunAltitudeRad);

    const kineticEnergy = 0.5 * currentMass * sSpdFE ** 2;
    const mechanicalPower = currentMass * sSpdFE * accel_long;

    // --- 3. MISE À JOUR DU DOM (Affichage) ---
    
    // ... (Mise à jour de la Section Contrôle/Vitesse/Distance/GPS/Dynamique, inchangée) ...
    
    // NOUVEAU : Affichage des Grandeurs Avancées (Bloc 2/4)
    if ($('lorentz-factor')) $('lorentz-factor').textContent = advancedPhysics.lorentzFactor.toExponential(4);
    if ($('time-dilation-speed')) $('time-dilation-speed').textContent = advancedPhysics.timeDilationSpeed.toFixed(3) + ' ns/j';
    if ($('grav-dilation')) $('grav-dilation').textContent = advancedPhysics.gravitationalDilation.toFixed(3) + ' ns/j';
    if ($('energy-relativistic')) $('energy-relativistic').textContent = advancedPhysics.energyRelativistic.toExponential(3) + ' J';
    if ($('energy-rest-mass')) $('energy-rest-mass').textContent = advancedPhysics.E0.toExponential(3) + ' J';
    if ($('momentum')) $('momentum').textContent = advancedPhysics.momentum.toFixed(2) + ' kg·m/s';
    if ($('mach-number')) $('mach-number').textContent = advancedPhysics.machNumber.toFixed(4);
    if ($('reynolds-number')) $('reynolds-number').textContent = advancedPhysics.reynoldsNumber.toExponential(2);
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = advancedPhysics.dynamicPressure.toFixed(2) + ' Pa';
    if ($('drag-force')) $('drag-force').textContent = advancedPhysics.dragForce.toFixed(2) + ' N';
    if ($('coriolis-force')) $('coriolis-force').textContent = advancedPhysics.coriolisForce.toExponential(2) + ' N';
    if ($('geopotential-alt')) $('geopotential-alt').textContent = advancedPhysics.geopotentialAltitude.toFixed(2) + ' m';
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = advancedPhysics.altSigma.toFixed(3) + ' m (σ)';
    if ($('Rs-object')) $('Rs-object').textContent = advancedPhysics.Rs_object.toExponential(3) + ' m';
    
    // NOUVEAU : Affichage des Grandeurs SVT/Météo (Bloc 3/4)
    if ($('abs-humidity-sim')) $('abs-humidity-sim').textContent = lastWeatherData ? lastWeatherData.absoluteHumidity.toFixed(3) + ' g/m³' : 'N/A';
    if ($('wet-bulb-temp-sim')) $('wet-bulb-temp-sim').textContent = bioSVT.wetBulbTemp.toFixed(1) + ' °C';
    if ($('CAPE-sim')) $('CAPE-sim').textContent = bioSVT.CAPE_sim.toFixed(0) + ' J/kg';
    if ($('O2-saturation-sim')) $('O2-saturation-sim').textContent = bioSVT.O2SaturationClamped.toFixed(1) + ' %';
    if ($('photosynthesis-rate-sim')) $('photosynthesis-rate-sim').textContent = bioSVT.photosynthesisRate.toExponential(2) + ' mol/s';
    if ($('respiration-rate-sim')) $('respiration-rate-sim').textContent = bioSVT.respirationRate.toExponential(3) + ' (taux)';
    if ($('solar-irradiance')) $('solar-irradiance').textContent = bioSVT.solarIrradiance.toFixed(1) + ' W/m²';
    if ($('radiation-pressure')) $('radiation-pressure').textContent = bioSVT.radiationPressure.toExponential(2) + ' Pa';
    
    // ... (Mise à jour de la Section IMU/Kalman, inchangée) ...

    // --- 4. SAUVEGARDE & MISE À JOUR CARTE ---
    lPos = pos; 
    lPos.speedMS_3D = spd3D_raw; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 

    updateMap(lat, lon, accRaw)
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); // Initialise la carte

    // --- Initialisation des Contrôles (Mass, Body, Env, etc.) ---
    // ... (Logique d'initialisation des contrôles, inchangée) ...
    
    // --- DÉMARRAGE DU SYSTÈME ---
    
    const initVals = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    G_ACC = initVals.G_ACC;
    R_ALT_CENTER_REF = initVals.R_ALT_CENTER_REF;
    
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        startGPS(); 
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = lat || 43.296; 
            const currentLon = lon || 5.370;
            
            // Met à jour l'astro (soleil, lune, horloge) (depuis astro_weather.js)
            if (typeof updateAstro === 'function') {
                const astro = updateAstro(currentLat, currentLon, lServH, lLocH);
                if (astro) sunAltitudeRad = astro.sunAltitudeRad;
            }
            
            // Resynchronise l'horloge NTP toutes les 60 secondes
            if (Math.floor(Date.now() / 1000) % 60 === 0) {
                 syncH(lServH, lLocH).then(newTimes => {
                    lServH = newTimes.lServH;
                    lLocH = newTimes.lLocH;
                 });
            }
            
            // Récupère la météo (si GPS actif) (depuis astro_weather.js)
            if (lat && lon && !emergencyStopActive && typeof fetchWeather === 'function') {
                fetchWeather(lat, lon, kAlt).then(data => {
                    if (data) {
                        lastWeatherData = data;
                        
                        // Met à jour le DOM météo
                        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                        // ... (Mise à jour des autres champs météo) ...
                    }
                });
            }
            
            // Met à jour l'horloge locale (NTP)
            const now = getCDate(lServH, lLocH);
            if (now) {
                // ... (Mise à jour des champs de temps) ...
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    }
});
