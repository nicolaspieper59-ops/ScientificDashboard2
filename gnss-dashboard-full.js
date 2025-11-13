// =================================================================
// BLOC 1/3 : ekf_logic.js
// Constantes de base, filtres EKF (Vitesse/Altitude) et fonctions de calcul physique/mathématique.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;      // Rapport des chaleurs spécifiques de l'air
const G_CONST = 6.67430e-11; // Constante gravitationnelle (N·m²/kg²)

let C_S_LOCAL = 343;        // Vitesse du son (m/s) - Mis à jour par la météo
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const M_EARTH = 5.972e24;   // Masse de la Terre (kg)

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE) ---
const Q_NOISE = 0.1;        // Bruit de processus
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement"
const ALT_TH = -50;         // Seuil d'altitude "Sous-sol"
const MAX_PLAUSIBLE_ACCEL = 20.0; // Anti-spike (m/s²)
const R_CENTER_EARTH = 6371000; // Rayon moyen pour le calcul de distance au centre

// --- SEUILS ZUPT (Zero Velocity Update) ---
const ZUPT_RAW_THRESHOLD = 1.0;     // Vitesse brute max (m/s)
const ZUPT_ACCEL_THRESHOLD = 0.5;   // Accélération max (m/s²)

// --- PARAMÈTRES EKF (ALTITUDE) ---
const Q_ALT_NOISE = 0.1;
const R_ALT_MIN = 0.1;

// --- CORRECTION ALTITUDE BAROMÉTRIQUE (Réel/Simulé) ---
const REF_P_HPA = 1013.25; // Pression de référence au niveau de la mer (hPa)

// --- FACTEURS ENVIRONNEMENTAUX (POUR R) ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel (Capteurs Priorisés)' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment (Capteurs Priorisés)' },
};

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre', M: M_EARTH },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune', M: 7.34767e22 },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars', M: 6.417e23 },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale', M: 0 }
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
    const data = CELESTIAL_DATA[bodyKey];
    if (!data) return 0;

    const M = data.M;
    const R_base = data.R;
    
    // Gravité basée sur la loi de la gravitation
    const g_local = G_CONST * M / (R_base + alt)**2;
    
    return g_local; 
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
            G_ACC_local = data.G; // Utilisation de la valeur de base pour l'affichage initial
            R_ALT_CENTER_REF_local = data.R;
        }
    }
    
    // Retourne les nouvelles valeurs à stocker globalement
    return { G_ACC: G_ACC_local, R_ALT_CENTER_REF: R_ALT_CENTER_REF_local };
}

/** Calcule le Facteur de Rapport de Mouvement (MRF) - Rapport Distance au centre de la Terre. */
function calculateMRF(alt_center, r_center, mode) {
    // alt_center : distance au centre de la Terre (R_center + altitude)
    // r_center : Rayon de référence (R_E_BASE)
    
    if (mode === 'UNDERGROUND') {
         // Rapport Distance: (Distance au centre au point actuel) / (Rayon terrestre de base)
        return (alt_center / r_center).toFixed(3); 
    }
    // Pour l'air ou la surface, le ratio est 1.0
    return 1.0; 
}

/** Calcule la vitesse du son corrigée par la température locale (m/s) */
function calculateSpeedOfSound(tempK) {
    if (tempK === null || tempK === 0) return 343;
    // Formule: C_S = sqrt(gamma * R_air * T)
    return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
}

/** Calcule la distance maximale visible jusqu'à l'horizon (m) */
function calculateDistanceToHorizon(altitude_m) {
    if (altitude_m === null || altitude_m < 0) return 0;
    // Formule: d = sqrt(2 * R_terre * h + h^2)
    return Math.sqrt(2 * R_E_BASE * altitude_m + altitude_m**2);
}

// --- FONCTIONS DE FILTRAGE (EKF) ---

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

/** * Applique le filtre de Kalman à l'Altitude. 
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
        // Si sous-sol, on augmente R, sauf si l'environnement est 'CONCRETE' ou 'METAL' 
        // où l'on suppose une meilleure fusion capteur (EKF)
        if (selectedEnv !== 'CONCRETE' && selectedEnvironment !== 'METAL') {
            R *= 2.0; 
        } else {
            R *= 0.5; // Moins de bruit perçu par l'EKF, car on fait confiance aux capteurs IMU/Baro (conceptuel)
        }
    } 

    return Math.max(R_MIN, Math.min(R_MAX, R)); 
}


/** Calcule l'Altitude Corrigée Barométrique (m) */
function calculateBaroCorrectedAlt(P_hPa, T_K, alt_ekf_m) {
    if (P_hPa === null || T_K === null || alt_ekf_m === null) return null;

    // Cette fonction simule la correction barométrique par rapport à la pression de référence (Niveau de la mer)
    const P_Pa = P_hPa * 100;
    const P_ref_Pa = REF_P_HPA * 100;
    const L = 0.0065; // Taux de déperdition (K/m)
    const T0 = 288.15; // Température standard au niveau de la mer (K)
    const g = CELESTIAL_DATA.EARTH.G; // Gravité
    
    let baro_alt = 0;
    if (P_Pa >= P_ref_Pa) {
        // Formule de nivellement simplifiée
        baro_alt = (T0 / L) * (1 - Math.pow(P_Pa / P_ref_Pa, (R_AIR * L) / g));
    } else {
        // Utilisation de la formule hypsométrique pour les plus grandes altitudes (simplifié)
        baro_alt = ((Math.pow(P_ref_Pa / P_Pa, 1 / 5.257) - 1) / 0.000022557).toFixed(2);
    }
    
    return parseFloat(baro_alt).toFixed(2);
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
            if (document.getElementById('local-time')) document.getElementById('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
            if (document.getElementById('date-display')) document.getElementById('date-display').textContent = now.toLocaleDateString('fr-FR');
            
            // Met à jour le statut
            if (document.getElementById('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                 document.getElementById('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
            }
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
            
            // Autres données (Calculées)
            // Humidité Absolue
            const absolute_humidity = (h_frac * 17.625 * 243.04 * Math.exp((17.625 * tempC) / (243.04 + tempC))) / ((243.04 + tempC) * 1000);
            const o2_rate = 20.9; // % Vol. d'oxygène dans l'air (standard)

            // Stocke les données pour le DOM et le filtre
            weatherData = {
                tempC: tempC,
                pressure_hPa: pressure_hPa,
                humidity_perc: humidity_perc,
                tempK: tempK,
                air_density: air_density,
                dew_point: dew_point,
                absolute_humidity: absolute_humidity,
                o2_rate: o2_rate
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

/** * Calcule les temps solaires (TST, MST, EOT) et les dates solaires. 
 */
function getSolarTime(date, lon) {
    if (date === null || lon === null || isNaN(lon)) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', NoonSolar: 'N/A', DateSolarMean: 'N/A', DateSolarTrue: 'N/A' };
    
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

    // Date Solaire Moyenne (basée sur MST)
    const date_solar_mean_ms = date.valueOf() + mst_offset_ms;
    const date_solar_mean = new Date(date_solar_mean_ms);

    // Date Solaire Vraie (basée sur TST)
    const date_solar_true_ms = date.valueOf() + mst_offset_ms + eot_ms;
    const date_solar_true = new Date(date_solar_true_ms);

    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    
    const toDateString = (d) => {
        let day = String(d.getUTCDate()).padStart(2, '0');
        let month = String(d.getUTCMonth() + 1).padStart(2, '0');
        let year = d.getUTCFullYear();
        return `${day}/${month}/${year}`;
    }

    // Calcul de l'heure UTC du Midi Solaire (pour l'affichage)
    const noon_solar_utc_ms = (J_transit - Math.floor(J_transit)) * dayMs;

    return { 
        TST: toTimeString(tst_ms), 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(2),
        ECL_LONG: (L * R2D).toFixed(2),
        NoonSolar: toTimeString(noon_solar_utc_ms), // MIDI SOLAIRE LOCAL en H:M:S UTC
        DateSolarMean: toDateString(date_solar_mean),
        DateSolarTrue: toDateString(date_solar_true)
    };
}

/** Calcule le temps Minecraft. */
function getMinecraftTime(date) {
    if (date === null) return '00:00';
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
    // Décalage de 6 heures pour faire correspondre le minuit UTC à 18:00 MC (début du jour MC)
    const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS + 6 * 3600000) % MC_DAY_MS; 
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;
    };
    return toTimeString(mcTimeMs);
}

/** Retourne le nom et le pourcentage de la phase lunaire */
function getMoonPhaseName(illum) {
    const phase = illum.phase;
    const fraction = illum.fraction;
    let name = '';
    
    if (phase < 0.03 || phase > 0.97) name = "Nouvelle Lune 🌑";
    else if (phase < 0.23) name = "Premier Croissant 🌒";
    else if (phase < 0.27) name = "Premier Quartier 🌓";
    else if (phase < 0.48) name = "Gibbeuse Croissante 🌔";
    else if (phase < 0.52) name = "Pleine Lune 🌕";
    else if (phase < 0.73) name = "Gibbeuse Décroissante 🌖";
    else if (phase < 0.77) name = "Dernier Quartier 🌗";
    else name = "Dernier Croissant 🌘";
    
    return { name, percent: (fraction * 100).toFixed(1) };
}

/** Convertit les heures de lever/coucher du soleil/lune aux formats demandés. */
function convertRiseSetTime(date, riseSetTime, lon, eot_min) {
    if (!riseSetTime) return 'N/A';
    
    const dateUtc = new Date(riseSetTime);
    const msSinceMidnightUtc = dateUtc.getUTCHours() * 3600000 + dateUtc.getUTCMinutes() * 60000 + dateUtc.getUTCSeconds() * 1000;

    // 1. Heure Locale (basée sur le fuseau horaire du navigateur)
    const localTime = dateUtc.toLocaleTimeString('fr-FR', { timeZoneName: 'short', hour12: false });
    
    // 2. Heure Solaire Moyenne (MST)
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUtc + mst_offset_ms + dayMs) % dayMs;
    const mstTime = new Date(mst_ms).toISOString().substring(11, 19); // HH:MM:SS
    
    // 3. Heure Solaire Vraie (TST)
    const eot_ms = eot_min * 60000;
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs;
    const tstTime = new Date(tst_ms).toISOString().substring(11, 19);
    
    return {
        local: localTime,
        mean: mstTime.substring(0, 5), // HH:MM
        true: tstTime.substring(0, 5)  // HH:MM
    };
}


/** Met à jour l'horloge visuelle et les couleurs du corps (Day/Night) */
function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const $ = id => document.getElementById(id); 
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const clockEl = $('minecraft-clock');
    const biomeHalf = $('biome-half');

    if (!sunEl || !moonEl || !clockEl || !biomeHalf) return;
    
    // 1. Position du Soleil
    if (sunPos) {
        const altDeg = sunPos.altitude * R2D;
        const aziDeg = (sunPos.azimuth * R2D + 180) % 360;
        
        // La rotation de l'élément entier simule l'orbite complète (rotation dans le plan du disque)
        // Position du Soleil (Utiliser la valeur de l'Azimut pour la rotation de l'orbite)
        sunEl.style.transform = `rotate(${aziDeg}deg)`; 
        // L'icône est positionnée sur le bord en fonction de l'altitude
        const sunIcon = sunEl.querySelector('.sun-icon');
        // Simple mapping de -90 à 90 degrés d'altitude à une position de 0% (haut) à 100% (bas)
        const alt_percent = (altDeg + 90) / 180 * 100;
        sunIcon.style.top = `${100 - alt_percent}%`;
        
        
        // 2. Position de la Lune (Utiliser le même principe)
        moonEl.style.transform = `rotate(${moonPos.azimuth * R2D + 180}deg)`; 
        const moonIcon = moonEl.querySelector('.moon-icon');
        const moon_alt_percent = (moonPos.altitude * R2D + 90) / 180 * 100;
        moonIcon.style.top = `${100 - moon_alt_percent}%`;
        
        // 3. Mise à jour du Ciel & Biome
        let sky_class = '';
        let status = 'Position Solaire Indisponible';
        let biome_color = '#795548'; // Couleur de biome par défaut (terre)

        if (sunTimes) {
            if (now > sunTimes.sunset || now < sunTimes.sunrise) {
                // Nuit / Crepuscule
                sky_class = (altDeg > -10) ? 'sky-night-light' : 'sky-night';
                status = 'Nuit 🌙';
                biome_color = '#3e2723'; // Couleur de la nuit (sous-sol sombre)
            } else if (now > sunTimes.dusk || now < sunTimes.dawn) {
                 // Crepuscule/Aube
                sky_class = 'sky-sunset';
                status = 'Crépuscule/Aube 🌅';
                biome_color = '#d2691e'; // Couleur du coucher de soleil/aube
            } else {
                sky_class = 'sky-day';
                status = 'Jour ☀️';
                biome_color = '#8BC34A'; // Couleur de jour (herbe)
            }
        }
        
        // Appliquer la classe ciel au body et à l'horloge
        document.body.className = document.body.className.split(' ').filter(c => !c.startsWith('sky-')).join(' ');
        document.body.classList.add(sky_class);
        clockEl.className = clockEl.className.split(' ').filter(c => !c.startsWith('sky-')).join(' ');
        clockEl.classList.add(sky_class);
        
        // Appliquer la couleur du biome à la moitié inférieure
        biomeHalf.style.backgroundColor = biome_color;
        
        if ($('clock-status')) $('clock-status').textContent = status;
    } else {
        if ($('clock-status')) $('clock-status').textContent = 'Position Solaire Indisponible';
    }
        }
// =================================================================
// BLOC 3/3 : main_logic.js
// Variables d'état, gestion des capteurs, DOM, et boucle de mise à jour.
// =================================================================

// --- VARIABLES DE TEMPS ET MISES À JOUR ---
const DOM_FAST_UPDATE_MS = 100;   // Mise à jour rapide (10 Hz)
const DOM_SLOW_UPDATE_MS = 1000;  // Mise à jour lente (1 sec)
let lastMapUpdate = 0;
const MAP_UPDATE_INTERVAL = 3000; // Rafraîchir la vue carte toutes les 3 sec si en mouvement

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000;
let timeMoving = 0, totalSessionTime = 0; // Ajout de totalSessionTime
let lServH = null, lLocH = null;
let lastFSpeed = 0;
let kAlt = null;
let kAltUncert = 10;
let currentGPSMode = 'HIGH_FREQ';
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL';
let currentMass = 70.0;
let R_FACTOR_RATIO = 1.0;
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0;
let gpsAccuracyOverride = 0.0;
let isXRayMode = false;

// Données externes
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 

// NOUVEAU : Variables IMU/Compteurs
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let real_mag_x = 0, real_mag_y = 0, real_mag_z = 0;
let sensor_sound_max = 0, sensor_light_max = 0, sensor_magnetic_max = 0;
let sensor_sound_avg = 0, sensor_light_avg = 0, sensor_magnetic_avg = 0;


// Objets Map
let map, marker, circle; 

// --- FONCTION UTILITAIRE DOM ---
const $ = id => document.getElementById(id);

// --- GESTION DES CAPTEURS IMU (Simulation pour l'exemple) ---
function updateIMU(pos, spd3D_raw, dt) {
    // Simulation simple pour l'exemple, en réalité utilise deviceorientation/devicemotion events
    real_accel_x = parseFloat((Math.random() * 0.2 - 0.1).toFixed(2));
    real_accel_y = parseFloat((Math.random() * 0.2 - 0.1).toFixed(2));
    real_accel_z = parseFloat((Math.random() * 0.2 - 0.1 + 9.8).toFixed(2));
    
    // Capteur magnétique pour le cap (Capteur Magnétomètre)
    real_mag_x = parseFloat((Math.random() * 10 - 5).toFixed(1));
    real_mag_y = parseFloat((Math.random() * 10 - 5).toFixed(1));
    real_mag_z = parseFloat((Math.random() * 10 - 5).toFixed(1));
    
    // Simulation du cap (basée sur le magnétomètre)
    const heading_mag = (Math.atan2(real_mag_y, real_mag_x) * R2D + 360) % 360;

    if ($('accel-x')) $('accel-x').textContent = `${real_accel_x.toFixed(2)} m/s²`;
    if ($('accel-y')) $('accel-y').textContent = `${real_accel_y.toFixed(2)} m/s²`;
    if ($('accel-z')) $('accel-z').textContent = `${(real_accel_z - 9.8).toFixed(2)} m/s²`; // Accélération linéaire (sans gravité)
    
    if ($('mag-x')) $('mag-x').textContent = `${real_mag_x.toFixed(1)} µT`;
    if ($('mag-y')) $('mag-y').textContent = `${real_mag_y.toFixed(1)} µT`;
    if ($('mag-z')) $('mag-z').textContent = `${real_mag_z.toFixed(1)} µT`;
    if ($('heading-display')) $('heading-display').textContent = `${heading_mag.toFixed(1)}° (Mag.)`;

    // Mise à jour des compteurs (Simulation)
    const sound_raw = Math.random() * (spd3D_raw > 1 ? 90 : 50); // dBA
    const light_raw = Math.random() * (spd3D_raw > 1 ? 500 : 100); // lux
    const magnetic_raw = Math.abs(real_mag_x) + Math.abs(real_mag_y) + Math.abs(real_mag_z); // µT

    sensor_sound_max = Math.max(sensor_sound_max, sound_raw);
    sensor_light_max = Math.max(sensor_light_max, light_raw);
    sensor_magnetic_max = Math.max(sensor_magnetic_max, magnetic_raw);
    
    // Simplification : Moyenne glissante
    sensor_sound_avg = (sensor_sound_avg * 9 + sound_raw) / 10;
    sensor_light_avg = (sensor_light_avg * 9 + light_raw) / 10;
    sensor_magnetic_avg = (sensor_magnetic_avg * 9 + magnetic_raw) / 10;

    if ($('sound-counter')) $('sound-counter').textContent = `${sensor_sound_max.toFixed(1)} / ${sensor_sound_avg.toFixed(1)} dBA`;
    if ($('light-counter')) $('light-counter').textContent = `${sensor_light_max.toFixed(0)} / ${sensor_light_avg.toFixed(0)} lux`;
    if ($('magnetic-counter')) $('magnetic-counter').textContent = `${sensor_magnetic_max.toFixed(1)} / ${sensor_magnetic_avg.toFixed(1)} µT`;

    return heading_mag; // Retourne le cap magnétique pour une éventuelle fusion avec le cap GPS
}


// --- GESTION DE LA CARTE ---

function initMap() {
    try {
        if (typeof L !== 'undefined') {
            map = L.map('map').setView([0, 0], 2);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { 
                attribution: '© OpenStreetMap contributors',
                // Ajout d'une inversion pour le mode nuit
                id: 'osm-dark', 
                maxZoom: 19
            }).addTo(map);
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
        navigator.geolocation.clearWatch(wID);
    }
    const interval = (mode === 'HIGH_FREQ') ? 100 : 1000;
    wID = navigator.geolocation.watchPosition(updateDisp, handleError, {
        enableHighAccuracy: true,
        timeout: 5000,
        maximumAge: 0
    });
    $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    $('speed-status-text').textContent = 'Signal GPS actif...';
    console.log(`Mode GPS défini: ${mode} (${interval}ms)`);
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
    $('speed-status-text').textContent = 'GPS Arrêté (Capteurs actifs)';
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Arrêté';
}

function handleError(error) {
    console.error(`Erreur GPS (${error.code}): ${error.message}`);
    $('speed-status-text').textContent = `Erreur GPS: ${error.message}`;
}

// --- LOGIQUE DE MISE À JOUR GPS (updateDisp) // ===========================================
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
    
    const cTime = now.getTime();
    if (sTime === null) { sTime = cTime; }
    
    const dt = lPos ? (cTime - lPos.cTime) / 1000 : DOM_FAST_UPDATE_MS / 1000; // Delta temps
    
    totalSessionTime = (cTime - sTime) / 1000; // Temps total écoulé
    if ($('elapsed-time')) $('elapsed-time').textContent = `${totalSessionTime.toFixed(2)} s`;


    if (gpsAccuracyOverride > 0.0) {
        accRaw = gpsAccuracyOverride;
    }
    
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
        R_dyn = 1e9; // Confiance GPS nulle
    } else {
        modeStatus = 'Signal GPS OK';
        if ($('gps-precision')) $('gps-precision').textContent = `🟢 ${accRaw.toFixed(2)} m`;
    }
    
    // --- 3. CALCUL DE VITESSE BRUTE 3D et DISTANCE ---
    let spd3D_raw = 0.0;
    let distance_moved = 0.0;
    
    if (lPos !== null) {
        const d2D = dist(lPos.lat, lPos.lon, cLat, cLon);
        const dAlt = altRaw !== null && lPos.altRaw !== null ? Math.abs(altRaw - lPos.altRaw) : 0;
        distance_moved = Math.sqrt(d2D * d2D + dAlt * dAlt);
        
        if (dt > 0) {
            spd3D_raw = distance_moved / dt;
        } else {
            spd3D_raw = 0.0;
        }
        
        // Anti-spike sur la vitesse brute
        if (lPos.speedMS_3D !== undefined && dt > 0.05) {
            const lastRawSpd = lPos.speedMS_3D;
            const accelSpike = Math.abs(spd3D_raw - lastRawSpd) / dt;
            if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
                console.warn(`Spike détecté: ${accelSpike.toFixed(2)} m/s². Correction appliquée.`);
                const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
                spd3D_raw = (spd3D_raw > lastRawSpd) ? (lastRawSpd + maxPlausibleChange) : (lastRawSpd - maxPlausibleChange);
            }
        }
        
        // Accumuler la distance totale (3D)
        distM += distance_moved;
        
        // Mettre à jour le temps de mouvement si la vitesse brute 3D est supérieure au seuil minimal
        if (spd3D_raw > MIN_SPD) {
            timeMoving += dt;
        }
    }
    
    // --- 4. MISE À JOUR IMU ET CAP ---
    const heading_mag = updateIMU(pos, spd3D_raw, dt);
    const final_heading = headingRaw !== null ? headingRaw.toFixed(1) + '° (GPS)' : heading_mag.toFixed(1) + '° (Mag.)';
    if ($('heading-display')) $('heading-display').textContent = final_heading;

    // --- 5. CALCULS D'ACCÉLÉRATION ET ALTITUDE BAROMÉTRIQUE ---
    let alt_center_dist = R_CENTER_EARTH + (kAlt || 0); // Distance au centre de la Terre
    let accel_alt_provisional = 0;
    let accel_long_provisional = 0;
    
    if (lPos && lPos.kAlt_old !== undefined && dt > 0.05) {
        accel_alt_provisional = (kAlt - lPos.kAlt_old) / dt; // Vitesse verticale
    } 
    if (lPos && lPos.speedMS_3D !== undefined && dt > 0.05) {
        accel_long_provisional = (spd3D_raw - lPos.speedMS_3D) / dt; // Accélération longitudinale
    }
    
    // Calcul de l'altitude corrigée (utilise la dernière pression météo)
    const alt_corrected_baro = calculateBaroCorrectedAlt(lastP_hPa, lastT_K, kAlt);
    if ($('alt-corrected-baro')) $('alt-corrected-baro').textContent = alt_corrected_baro !== null ? `${alt_corrected_baro} m` : 'N/A';
    
    // Mise à jour de la gravité locale (basée sur l'altitude EKF)
    const gravity_local = getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity);
    if ($('gravity-local')) $('gravity-local').textContent = `${gravity_local.toFixed(4)} m/s²`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long_provisional.toFixed(2)} m/s²`;

    // --- 6. LOGIQUE ZUPT (Zero Velocity Update) ---
    let spd_kalman_input = spd3D_raw;
    let R_kalman_input = R_dyn;
    const isPlausiblyStopped = (
        spd3D_raw < ZUPT_RAW_THRESHOLD && 
        Math.abs(accel_long_provisional) < ZUPT_ACCEL_THRESHOLD && 
        R_dyn < R_MAX 
    );
    
    // L'EKF prend le dessus dans les tunnels/grottes si le signal GPS est mauvais.
    // L'EKF fusionnera les accélérations IMU (real_accel_x/y/z) avec la prédiction.
    if ((selectedEnvironment === 'CONCRETE' || selectedEnvironment === 'METAL') && isSignalLost) {
        modeStatus = '🛰️ Capteurs EKF (Grotte/Tunnel)';
        R_kalman_input = R_MIN * 0.1; // Confiance maximale dans l'EKF et IMU
    }
    
    if (isPlausiblyStopped) {
        spd_kalman_input = 0.0; // Forcer la mesure à 0 m/s
        R_kalman_input = R_MIN; // Confiance maximale dans la mesure ZUPT
        modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)';
    }

    // --- 7. FILTRE EKF VITESSE ---
    const { kSpd: kSpd_new, kUncert: kUncert_new } = kFilter(kSpd, kUncert, spd_kalman_input, dt, R_kalman_input, accel_long_provisional);
    kSpd = kSpd_new;
    kUncert = kUncert_new;
    
    // Mise à jour de la vitesse max (utilise la vitesse 3D instantanée, non filtrée)
    maxSpd = Math.max(maxSpd, spd3D_raw);
    
    // --- 8. FILTRE EKF ALTITUDE ---
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(kAlt, kAltUncert, altRaw, accRaw, dt);
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // --- 9. CALCULS RELATIVISTES ET PHYSIQUES AVANCÉS ---
    const v = kSpd;
    const c = C_L;
    const m = currentMass;
    
    const lorentz_factor = 1.0 / Math.sqrt(1 - (v / c)**2);
    const relativistic_energy = m * c**2 * lorentz_factor; // E = gamma * E0
    const rest_mass_energy = m * c**2; // E0 = mc^2
    const momentum = m * v * lorentz_factor; // p = gamma * m * v
    const sch_radius = (2 * G_CONST * m) / c**2; // Rayon de Schwarzschild
    
    // Dilatation du temps (ns/jour)
    const dilation_v = (lorentz_factor - 1.0) * dayMs * 1e-6; // en ns
    // Dilatation du temps gravitationnelle (simplification)
    const dilation_g = (1 - 1.0 / Math.sqrt(1 - (2 * gravity_local * kAlt / c**2))) * dayMs * 1e-6; 

    // Mettre à jour la vitesse du son (nécessite T_K de la météo)
    if (lastT_K !== null) {
        C_S_LOCAL = calculateSpeedOfSound(lastT_K);
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${C_S_LOCAL.toFixed(2)} m/s`;
    }
    
    // Pression dynamique (q = 0.5 * rho * v^2)
    const air_density = lastP_hPa !== null && lastT_K !== null ? (lastP_hPa * 100) / (R_AIR * lastT_K) : 1.225;
    const dynamic_pressure = 0.5 * air_density * v**2;
    
    // Force de Traînée (simplification Fd = q * Cd * A) - Assumons Cd*A = 1m²
    const drag_force = dynamic_pressure * 1.0; 
    
    // Force de Coriolis (simplification F_c = -2 * m * omega x v)
    const coriolis_force = 2 * m * OMEGA_EARTH * v * Math.cos(cLat * D2R);

    // --- 10. MISE À JOUR DU DOM (VITESSE ET PHYSIQUE) ---
    
    if ($('speed-status-text')) $('speed-status-text').textContent = modeStatus;
    if ($('speed-stable')) $('speed-stable').textContent = `${(kSpd * KMH_MS).toFixed(2)} km/h`;
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = `${(kSpd / 1000).toExponential(3)} km/s`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${kSpd.toFixed(3)} m/s`; 
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(2)} km/h`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd3D_raw.toFixed(3)} m/s`;
    
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(2)} km/h`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? `${(distM / timeMoving * KMH_MS).toFixed(2)} km/h` : '0.0 km/h';
    if ($('speed-avg-total')) $('speed-avg-total').textContent = totalSessionTime > 1 ? `${(distM / totalSessionTime * KMH_MS).toFixed(2)} km/h` : '0.0 km/h';

    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(v / C_S_LOCAL * 100).toFixed(2)} %`;
    if ($('mach-number')) $('mach-number').textContent = `${(v / C_S_LOCAL).toFixed(4)}`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(v / C_L * 100).toExponential(2)}%`;
    if ($('lorentz-factor')) $('lorentz-factor').textContent = `${lorentz_factor.toFixed(4)}`;
    
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    // Mise à jour du Rapport de Distance (MRF)
    const mrf_value = altRaw !== null ? calculateMRF(R_CENTER_EARTH + altRaw, R_CENTER_EARTH, altRaw < ALT_TH ? 'UNDERGROUND' : 'SURFACE') : 1.0;
    if ($('distance-ratio')) $('distance-ratio').textContent = mrf_value;
    
    if ($('distance-light-s')) $('distance-light-s').textContent = `${(distM / C_L).toExponential(2)} s`;
    if ($('distance-light-min')) $('distance-light-min').textContent = `${(distM / C_L / 60).toExponential(2)} min`;
    if ($('distance-horizon')) $('distance-horizon').textContent = `${(calculateDistanceToHorizon(kAlt || 0) / 1000).toFixed(2)} km`;
    
    // Mise à jour EKF & Debug
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kUncert_new.toFixed(3)} m²/s²`;
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = `${Math.sqrt(kAltUncert_new).toFixed(3)} m`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m²/s²`;
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Actif';
    if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    
    // Mise à jour Relativiste et Fluides
    if ($('time-dilation-v')) $('time-dilation-v').textContent = `${dilation_v.toExponential(2)} ns/j`;
    if ($('time-dilation-g')) $('time-dilation-g').textContent = `${dilation_g.toExponential(2)} ns/j`;
    if ($('relativistic-energy')) $('relativistic-energy').textContent = `${relativistic_energy.toExponential(2)} J`;
    if ($('rest-mass-energy')) $('rest-mass-energy').textContent = `${rest_mass_energy.toExponential(2)} J`;
    if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = `${sch_radius.toExponential(2)} m`;
    if ($('momentum')) $('momentum').textContent = `${momentum.toFixed(2)} kg·m/s`;
    
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = `${dynamic_pressure.toFixed(2)} Pa`;
    if ($('drag-force')) $('drag-force').textContent = `${drag_force.toFixed(2)} N`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis_force.toExponential(2)} N`;
    
    // Mise à jour Position
    lat = cLat; lon = cLon;
    if ($('lat-display')) $('lat-display').textContent = `${lat.toFixed(6)}`;
    if ($('lon-display')) $('lon-display').textContent = `${lon.toFixed(6)}`;
    if ($('alt-display')) $('alt-display').textContent = kAlt !== null ? `${kAlt.toFixed(2)} m` : 'N/A';
    
    // Sauvegarder l'état précédent
    lPos = { 
        cTime: cTime, lat: cLat, lon: cLon, altRaw: altRaw, accRaw: accRaw,
        speedMS_3D: spd3D_raw, kAlt_old: kAlt_new, kSpd_old: kSpd_new 
    };
    
    // Mise à jour de la carte
    updateMap(lat, lon, accRaw);
}

// --- FONCTION PRINCIPALE ASTRO (appelée par la boucle lente) ---
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
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min';
    if ($('ecl-long')) $('ecl-long').textContent = solarTimes.ECL_LONG + '°';

    if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${((sunPos.azimuth * R2D + 180) % 360).toFixed(2)}°`;
    
    // Lune
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)}%`;
    if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)}°`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${((moonPos.azimuth * R2D + 180) % 360).toFixed(2)}°`;
    
    updateClockVisualization(now, sunPos, moonPos, sunTimes);
        }
// Remarque : Le bloc précédent se terminait par la fermeture de la fonction updateAstro :
// updateClockVisualization(now, sunPos, moonPos, sunTimes);
// } // <-- Fermeture de la fonction updateAstro

// --- FONCTION D'INITIALISATION ET BOUCLES DE MISE À JOUR ---

/**
 * Boucle de mise à jour rapide (Fast Update Loop).
 * Met à jour l'IMU et l'heure locale rapidement.
 */
function domUpdateLoop() {
    // La fonction updateIMU est appelée pour simuler l'acquisition rapide des capteurs
    updateIMU(null, 0, DOM_FAST_UPDATE_MS / 1000); 
    
    // Récupère l'heure corrigée (NTP/Locale)
    const now = getCDate(lServH, lLocH);
    if (now) {
        if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    }
}

/**
 * Initialise l'application, configure les boucles et les écouteurs d'événements.
 */
function init() {
    // Initialise la carte Leaflet
    initMap();
    
    // Démarre la surveillance GPS/Geolocalisation
    setGPSMode(currentGPSMode);
    
    // Boucle rapide (ex: 50ms) pour l'IMU et l'affichage de l'heure
    setInterval(domUpdateLoop, DOM_FAST_UPDATE_MS); 

    // Boucle lente (ex: 1000ms) pour les calculs Astro et la météo
    setInterval(() => {
        // Appelle la fonction updateAstro uniquement si des coordonnées sont disponibles
        if (lat !== null && lon !== null) {
            updateAstro(lat, lon, lServH, lLocH);
        }
        // Appeler la mise à jour météo ici si elle est implémentée
        // updateWeather(); 
    }, DOM_SLOW_UPDATE_MS);

    // --- CONFIGURATION DES LISTENERS D'ÉVÉNEMENTS ---
    
    // MARCHE/ARRÊT GPS
    $('toggle-gps-btn').addEventListener('click', () => {
        if (wID !== null) stopGPS(); else setGPSMode(currentGPSMode);
    });
    
    // Sélection de fréquence
    $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    
    // Arrêt d'urgence
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '🚨 Arrêt d\'urgence: ACTIF 🔴' : '🛑 Arrêt d\'urgence: INACTIF 🟢';
        $('emergency-stop-btn').classList.toggle('active', emergencyStopActive);
    });
    
    // Mode Nuit
    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        // Logique pour inverser les couleurs de la carte Leaflet
        const mapEl = document.getElementById('map');
        if (mapEl) {
            const isDarkMode = document.body.classList.contains('dark-mode');
            mapEl.style.filter = isDarkMode ? 'invert(0.9) hue-rotate(180deg)' : 'none';
        }
    });

    // Reset des compteurs
    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; maxSpd = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    
    // Paramètres EKF/Physique
    $('gps-accuracy-override').addEventListener('input', (e) => { gpsAccuracyOverride = parseFloat(e.target.value); });
    $('mass-input').addEventListener('input', (e) => { currentMass = parseFloat(e.target.value); $('mass-display').textContent = currentMass.toFixed(3) + ' kg'; });
    $('celestial-body-select').addEventListener('change', (e) => { currentCelestialBody = e.target.value; });
    $('environment-select').addEventListener('change', (e) => { selectedEnvironment = e.target.value; /* Mise à jour de R_FACTOR_RATIO ici */ });
    
    // Bouton Rayons X pour l'horloge Minecraft
    $('xray-button').addEventListener('click', () => { 
        isXRayMode = !isXRayMode;
        document.getElementById('minecraft-clock').classList.toggle('x-ray', isXRayMode);
        $('xray-button').textContent = isXRayMode ? 'ON' : 'X';
    });
}

// Démarre l'application après le chargement complet du HTML
document.addEventListener('DOMContentLoaded', init);
