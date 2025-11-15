// =========================================================================
// GNSS SpaceTime Dashboard - FULL JS (V5.0)
// Fichier synchronisé avec index.html (7)
// =========================================================================

// =========================================================================
// BLOC 1/4 : Constantes, État Global & Fonctions Mathématiques/Physiques
// =========================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const KMS_MS = 1 / 1000;    // Conversion m/s vers km/s
const C_S_BASE = 340.29;    // Vitesse du son standard (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;      // Rapport des chaleurs spécifiques pour l'air
const KELVIN_OFFSET = 273.15; // Conversion Celsius vers Kelvin
const R_MAX = 500.0;        // Précision Max GPS (m) avant "Estimation Seule"

// --- PARAMÈTRES DU FILTRE DE KALMAN (TUNING AVIONIQUE) ---
const Q_NOISE = 0.5;        // Bruit de processus (Accélération)
const MIN_SPD = 0.05;       // Vitesse minimale pour considérer le mouvement (m/s)
const MIN_DT = 0.01;        // Intervalle de temps minimum (s)
const ZUPT_ACCEL_TOLERANCE = 0.5; // Tolérance d'accélération pour ZUPT (m/s²)

// --- PARAMÈTRES DYNAMIQUES PAR DÉFAUT ---
const DEFAULT_DRAG_COEF = 0.5; // Cd (e.g., pour un véhicule standard)
const DEFAULT_REF_AREA = 1.0; // A (m²)
const DEFAULT_MASS = 70.0; // kg

// --- CONSTANTES DE L'APPLICATION ---
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 250; // Boucle lente (Astro/Météo)
const DEFAULT_INIT_LAT = 45.749950;
const DEFAULT_INIT_LON = 4.850027;
const DEFAULT_INIT_ALT = 2.64;

// --- NOUVELLES CITATIONS DE STATUT ---
const STATUS_QUOTES = {
    WAITING_FOR_SYNC: "« Données en attente. »",
    OFFLINE_STORAGE: "« Mode Hors-Ligne: Sauvegarde active. »",
    EMERGENCY_STOP: "« Arrêt d'urgence. Session figée. »"
};

// --- STRUCTURES DE DONNÉES ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: 6371000, DISPLAY: 'Terre' },
    'MARS': { G: 3.72076, R: 3389500, DISPLAY: 'Mars' },
    'MOON': { G: 1.625, R: 1737400, DISPLAY: 'Lune' },
    'ROTATING': { G: 0.0, R: R_E_BASE, DISPLAY: 'Station Spatiale' }
};

const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' }
};

// --- ÉTAT DU FILTRE EKF ---
let currentEKFState = {
    lat: DEFAULT_INIT_LAT, lon: DEFAULT_INIT_LON, alt: DEFAULT_INIT_ALT,
    V_n: 0.0, V_e: 0.0, V_d: 0.0, 
    acc_est: 10.0, 
    P_pos: 100.0, P_vel: 0.625
};

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, domID = null, lPos = null, sTime = null;
let distM_3D = 0, maxSpd = 0, timeMoving = 0, lastUpdateTime = 0; 
// CORRECTION : Initialisation des heures de référence NTP pour la stabilité
let lServH = Date.now(), lLocH = Date.now(); 
let emergencyStopActive = false;
let currentMass = DEFAULT_MASS; 
let currentDragCoef = DEFAULT_DRAG_COEF, currentRefArea = DEFAULT_REF_AREA;
let angularVelocity = 0.0, gpsAccuracyOverride = 0.0; 
let G_ACC = CELESTIAL_DATA['EARTH'].G; R_ALT_CENTER_REF = R_E_BASE;            
let currentEnvFactor = 1.0;
let lastP_hPa = null, lastT_K = null, lastH_perc = null, lastAirDensity = null; 
let lastSpeedOfSound = C_S_BASE;
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let lastAccelLong = 0;
let map, marker, accuracyCircle;
let lastMapUpdate = 0;


// --- UTILITIES ---
const $ = id => document.getElementById(id);

// --- FONCTION DE GESTION DE L'HEURE RÉFÉRENCE ---
function getCDate(lServH, lLocH) {
    if (lServH === null || lLocH === null) {
        return new Date();
    }
    // Si lServH et lLocH sont initialisés (maintenant le cas), cette fonction est stable.
    const offset = lServH - lLocH;
    return new Date(Date.now() + offset);
}

/**
 * Vérifie et applique une classe CSS aux champs dont les données sont manquantes.
 */
function highlightMissingData() {
    const missingTexts = ['N/A', '--', 'INACTIF', 'ERREUR']; 
    // Sélectionne tous les spans qui affichent les valeurs
    const valueSpans = document.querySelectorAll('.data-point span:last-child');
    
    let quoteToUse;
    if (emergencyStopActive) {
        quoteToUse = STATUS_QUOTES.EMERGENCY_STOP;
    } else if (wID === null) {
        quoteToUse = STATUS_QUOTES.OFFLINE_STORAGE;
    } else {
        quoteToUse = STATUS_QUOTES.WAITING_FOR_SYNC;
    }

    valueSpans.forEach(span => {
        const currentText = span.textContent.trim().toUpperCase();
        let isMissing = missingTexts.some(mt => currentText.includes(mt));
        
        if (span.id.includes('emergency-stop')) return;
        
        // CORRECTION : S'assurer que les champs EKF/Dynamique initialisés ne sont pas considérés comme manquants
        if (currentText.includes('M/S') || currentText.includes('KG/M³') || currentText.includes('PA') || currentText.includes('N')) {
             isMissing = false; 
        }
        
        if (isMissing) {
            span.classList.add('value-missing');
            
            // Remplace N/A, --, INACTIF
            if (currentText === 'N/A' || currentText === '--' || currentText.includes('INACTIF')) {
                span.textContent = quoteToUse;
            }
            
        } else {
            span.classList.remove('value-missing');
        }
    });
                 }
// =========================================================================
// BLOC 2/4 : Fonctions EKF, Astro et Météo (Logique de Fusion)
// =========================================================================

async function syncH(lServH, lLocH) {
    return new Promise((resolve) => {
        const localTimeBefore = Date.now();
        const serverTime = Date.now() + 150; // Simule un décalage NTP de 150ms
        const localTimeAfter = Date.now();
        const latency = (localTimeAfter - localTimeBefore) / 2;
        
        const newServH = serverTime + latency;
        const newLocH = localTimeAfter;
        
        resolve({ lServH: newServH, lLocH: newLocH });
    });
}

async function fetchWeather(lat, lon) {
    try {
        // Simulation locale d'une requête météo standard
        const tempC = 19.0; 
        const pressure_hPa = 1012.0; 
        const humidity_perc = 82.0; 
        
        const tempK = tempC + KELVIN_OFFSET;
        const pressure_Pa = pressure_hPa * 100; 
        
        // Densité de l'air : rho = P / (R_air * T_K)
        const air_density = pressure_Pa / (R_AIR * tempK);
        
        // Vitesse du son : c = sqrt(gamma * R_air * T_K)
        const speed_of_sound = Math.sqrt(GAMMA_AIR * R_AIR * tempK);

        // Calcul du Point de Rosée
        const A = 17.27, B = 237.7;
        const alpha = ((A * tempC) / (B + tempC)) + Math.log(humidity_perc / 100);
        const dew_point = (B * alpha) / (A - alpha);

        return {
            tempC, tempK, pressure_hPa, humidity_perc, air_density,
            dew_point: dew_point.toFixed(1),
            status: "Dégagé",
            speed_of_sound: speed_of_sound
        };
    } catch (e) {
        console.error("Erreur de simulation de météo:", e);
        return null;
    }
}

// CORRECTION : Utilisation de MC_DAY_MS et correction de la logique de calcul de l'heure MC
function getMinecraftTime(now) {
    const MC_DAY_MS = 20 * 60 * 1000; 
    const midnightOffsetMs = 6 * 3600 * 1000; 
    const msSinceEpoch = now.getTime();
    
    let timeInCycle = (msSinceEpoch - midnightOffsetMs) % MC_DAY_MS;
    if (timeInCycle < 0) timeInCycle += MC_DAY_MS; 

    const msPerMcHour = MC_DAY_MS / 24; 
    
    const mcHours = Math.floor(timeInCycle / msPerMcHour);
    const mcMinutes = Math.floor((timeInCycle % msPerMcHour) / (msPerMcHour / 60));
    
    return `${String(mcHours).padStart(2, '0')}:${String(mcMinutes).padStart(2, '0')}`;
}

function updateAstro(lat, lon, lServH, lLocH) {
    // Vérification de la dépendance SunCalc (doit être chargée dans le HTML)
    if (typeof SunCalc === 'undefined' || lat === 0 || lon === 0) return null;
    
    const now = getCDate(lServH, lLocH); 
    const daySinceJ2000 = (now.getTime() - new Date(2000, 0, 1, 12, 0, 0).getTime()) / 86400000;
    
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    
    // --- CALCULS SOLAIRES AVANCÉS (EOT, TST, MST) ---
    // Correction de la formule EOT et ECL_LONG
    const M_deg = (357.5291 + 0.98560028 * daySinceJ2000); 
    const M_rad_trig = M_deg * D2R; 
    
    const L_deg = M_deg + 
                  (1.9148 * Math.sin(M_rad_trig)) + 
                  (0.0200 * Math.sin(2 * M_rad_trig)) + 
                  102.9372;
    
    let L_rad = (L_deg % 360) * D2R; 
    
    const E_rad = (23.4393 - 3.563e-7 * daySinceJ2000) * D2R;
    
    const RA_rad = Math.atan2(Math.cos(E_rad) * Math.sin(L_rad), Math.cos(L_rad));
    
    let EOT_deg_diff = (L_deg - RA_rad * R2D); 
    EOT_deg_diff = EOT_deg_diff % 360;
    if (EOT_deg_diff > 180) EOT_deg_diff -= 360;
    if (EOT_deg_diff < -180) EOT_deg_diff += 360;
    
    const EOT_min = EOT_deg_diff * 4; // Équation du temps en minutes

    const now_utc_h = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    
    const TST_hours = now_utc_h + (lon / 15) + (EOT_min / 60);
    const MST_hours = now_utc_h + (lon / 15);
    const noon_utc_h = 12.0 - (lon / 15) - (EOT_min / 60);

    const decimalToTime = (h) => {
        let total_seconds = h * 3600;
        if (total_seconds < 0) total_seconds += Math.ceil(-total_seconds / (24 * 3600)) * 24 * 3600;
        total_seconds = total_seconds % (24 * 3600); 
        
        const h_int = Math.floor(total_seconds / 3600);
        const m_int = Math.floor((total_seconds % 3600) / 60);
        const s_int = Math.floor(total_seconds % 60);
        return `${String(h_int).padStart(2, '0')}:${String(m_int).padStart(2, '0')}:${String(s_int).padStart(2, '0')}`;
    };
    
    // Détermination de l'état du ciel
    let skyStatus = '';
    const alt_deg = sunPos.altitude * R2D;
    
    if (alt_deg > 5) {
        skyStatus = 'sky-day'; 
    } else if (alt_deg > -0.833) {
        skyStatus = 'sky-sunset'; 
    } else if (alt_deg > -18) {
        skyStatus = 'sky-night-light'; 
    } else {
        skyStatus = 'sky-night'; 
    }


    return {
        now, sunPos, moonIllum, moonPos, sunTimes,
        solarTimes: {
            TST: decimalToTime(TST_hours), 
            MST: decimalToTime(MST_hours),
            EOT: EOT_min.toFixed(2), 
            ECL_LONG: (L_deg % 360).toFixed(2), 
            NoonSolar: decimalToTime(noon_utc_h)
        },
        skyStatus
    };
}

function getMoonPhaseName(phase) {
    if (phase < 0.06 || phase > 0.94) return "Nouvelle Lune";
    if (phase < 0.25) return "Premier Croissant";
    if (phase < 0.44) return "Premier Quartier";
    if (phase < 0.56) return "Pleine Lune";
    if (phase < 0.75) return "Gibbeuse Décroissante";
    if (phase < 0.94) return "Dernier Quartier";
    return "Phase Inconnue";
}


// --------------------------------------------------------------------------
// --- CORE LOGIC EKF (Simplified 21-State) ---
// --------------------------------------------------------------------------

function initEKF(lat, lon, alt, acc) {
    currentEKFState.lat = lat;
    currentEKFState.lon = lon;
    currentEKFState.alt = alt;
    currentEKFState.acc_est = acc;
    currentEKFState.V_n = 0.0;
    currentEKFState.V_e = 0.0;
    currentEKFState.V_d = 0.0;
    currentEKFState.P_pos = acc * acc; 
    currentEKFState.P_vel = Q_NOISE * 2; 
}

function predictEKF(dt, acc_imu, G_ACC, R_ALT_CENTER_REF) {
    const Vn = currentEKFState.V_n;
    const Ve = currentEKFState.V_e;
    const lat = currentEKFState.lat;

    // 1. Prédiction de l'État (X)
    const accel_n = acc_imu[0]; // Nous supposons que l'IMU est déjà aligné sur le Nord-Est-Bas
    const accel_e = acc_imu[1]; 
    const accel_d = acc_imu[2] - G_ACC; // acc_z - G

    currentEKFState.V_n += accel_n * dt;
    currentEKFState.V_e += accel_e * dt;
    currentEKFState.V_d += accel_d * dt;

    // Rayons de courbure locaux pour la conversion Vitesse -> Position (simplifié)
    const R_MERIDIAN = R_ALT_CENTER_REF; 
    const R_TRANSVERSE = R_ALT_CENTER_REF * Math.cos(lat * D2R); 

    currentEKFState.lat += (Vn * dt) / R_MERIDIAN * R2D;
    currentEKFState.lon += (Ve * dt) / R_TRANSVERSE * R2D;
    currentEKFState.alt -= currentEKFState.V_d * dt; 

    // 2. Prédiction de l'Incertitude (P)
    currentEKFState.P_pos += Q_NOISE * dt * 0.1; 
    currentEKFState.P_vel += Q_NOISE * dt; 
}

function updateEKF_GNSS(gnss_pos, gnss_vel, accRaw, altAcc) {
    const R_dynamic = accRaw * accRaw * currentEnvFactor; 
    
    // --- Correction de Position (Lat/Lon/Alt) ---
    const K_pos = currentEKFState.P_pos / (currentEKFState.P_pos + R_dynamic);
    
    currentEKFState.lat += K_pos * (gnss_pos.lat - currentEKFState.lat);
    currentEKFState.lon += K_pos * (gnss_pos.lon - currentEKFState.lon);
    
    const K_alt = currentEKFState.P_pos / (currentEKFState.P_pos + altAcc * altAcc);
    currentEKFState.alt += K_alt * (gnss_pos.alt - currentEKFState.alt);

    currentEKFState.P_pos = (1 - K_pos) * currentEKFState.P_pos;
    currentEKFState.acc_est = Math.sqrt(currentEKFState.P_pos); 

    // --- Correction de Vitesse (Vn, Ve, Vd) ---
    if (gnss_vel.V_n !== null && gnss_vel.V_n !== undefined) {
        const R_vel = R_dynamic * 0.1; 
        const K_vel = currentEKFState.P_vel / (currentEKFState.P_vel + R_vel);
        
        // CORRECTION V_n et V_e : Utilise la projection de la vitesse GNSS
        currentEKFState.V_n += K_vel * (gnss_vel.V_n - currentEKFState.V_n);
        currentEKFState.V_e += K_vel * (gnss_vel.V_e - currentEKFState.V_e); 
        
        // CORRECTION V_d : Force la vitesse verticale vers 0 si non mesurée
        currentEKFState.V_d += K_vel * (gnss_vel.V_d - currentEKFState.V_d); 

        currentEKFState.P_vel = (1 - K_vel) * currentEKFState.P_vel;
    }
}

function updateEKF_ZUPT() {
    currentEKFState.V_n = 0.0;
    currentEKFState.V_e = 0.0;
    currentEKFState.V_d = 0.0;
    currentEKFState.P_vel = Q_NOISE * 0.1; 
}

function getEKFVelocity3D() {
    return Math.sqrt(currentEKFState.V_n * currentEKFState.V_n + 
                     currentEKFState.V_e * currentEKFState.V_e + 
                     currentEKFState.V_d * currentEKFState.V_d);
}

function getVelocityUncertainty() { 
    return currentEKFState.P_vel; 
    }
// =========================================================================
// BLOC 3/4 : Gestion des Événements IMU & GPS
// =========================================================================

function handleIMU(event) {
    if (event.acceleration) {
        // L'accélération fournie par l'API est déjà corrigée de la gravité par certains navigateurs
        // Nous l'utilisons brute pour la prédiction EKF
        real_accel_x = event.acceleration.x || 0;
        real_accel_y = event.acceleration.y || 0;
        real_accel_z = event.acceleration.z || 0;
        
        $('accel-x').textContent = `${real_accel_x.toFixed(3)} m/s²`;
        $('accel-y').textContent = `${real_accel_y.toFixed(3)} m/s²`;
        $('accel-z').textContent = `${real_accel_z.toFixed(3)} m/s²`;
        $('imu-status').textContent = 'Actif (Accel)';
    }

    if (event.rotationRate) {
        const gyroX = event.rotationRate.alpha || 0;
        const gyroY = event.rotationRate.beta || 0;
        const gyroZ = event.rotationRate.gamma || 0;
        angularVelocity = Math.sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
        $('angular-speed').textContent = `${angularVelocity.toFixed(3)} rad/s`;
    }
}

function handlePosition(pos) {
    const cTimePos = pos.timestamp;
    // Utiliser MIN_DT pour éviter la division par zéro si timestamp est le même
    const dt = lastUpdateTime === 0 ? MIN_DT : Math.max(MIN_DT, (cTimePos - lastUpdateTime) / 1000); 
    lastUpdateTime = cTimePos;
    
    if (emergencyStopActive) {
        updateDisp(pos, dt);
        return;
    }

    // --- MISE À JOUR EKF (PRÉDICTION) ---
    // Les accélérations sont supposées être dans le cadre local (Nord, Est, Bas)
    const accel_imu = [real_accel_x, real_accel_y, real_accel_z]; 
    predictEKF(dt, accel_imu, G_ACC, R_ALT_CENTER_REF);

    if (lPos === null) {
        initEKF(pos.coords.latitude, pos.coords.longitude, pos.coords.altitude || DEFAULT_INIT_ALT, pos.coords.accuracy);
        sTime = Date.now();
    } else {
        // Calcul de distance (3D)
        const d_lat = (pos.coords.latitude - lPos.coords.latitude) * R_E_BASE * D2R;
        const d_lon = (pos.coords.longitude - lPos.coords.longitude) * R_E_BASE * D2R * Math.cos(pos.coords.latitude * D2R);
        const d_alt = (pos.coords.altitude || lPos.coords.altitude || currentEKFState.alt) - (lPos.coords.altitude || currentEKFState.alt);
        const d_3d = Math.sqrt(d_lat * d_lat + d_lon * d_lon + d_alt * d_alt);
        
        distM_3D += d_3d;

        // --- ZUPT (Zero-Velocity Update) ---
        const currentAccelMagnitude = Math.sqrt(real_accel_x*real_accel_x + real_accel_y*real_accel_y + real_accel_z*real_accel_z);
        if (getEKFVelocity3D() < MIN_SPD && currentAccelMagnitude < ZUPT_ACCEL_TOLERANCE) {
            updateEKF_ZUPT();
        }

        // Temps de mouvement
        if (getEKFVelocity3D() >= MIN_SPD) {
            timeMoving += dt;
        }
    }

    // --- MISE À JOUR EKF (CORRECTION GPS) ---
    const currentAcc = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : pos.coords.accuracy;

    if (currentAcc < R_MAX) {
        const gnss_pos = { lat: pos.coords.latitude, lon: pos.coords.longitude, alt: pos.coords.altitude || currentEKFState.alt };
        
        let V_gnss_n = 0.0;
        let V_gnss_e = 0.0;
        const rawSpeed = pos.coords.speed || 0;
        
        // CORRECTION : Projection de la vitesse GPS sur les axes Nord/Est
        if (pos.coords.heading !== null && pos.coords.heading !== undefined) {
            const headingRad = pos.coords.heading * D2R;
            V_gnss_n = rawSpeed * Math.cos(headingRad);
            V_gnss_e = rawSpeed * Math.sin(headingRad);
        } else {
            // Si le cap est manquant, correction minimale sur la composante V_n
            V_gnss_n = rawSpeed;
            V_gnss_e = 0.0; 
        }

        const gnss_vel = { V_n: V_gnss_n, V_e: V_gnss_e, V_d: 0.0 }; // V_d GNSS est 0
        updateEKF_GNSS(gnss_pos, gnss_vel, currentAcc, 10.0);
        
        $('gps-status-dr').textContent = 'Actif (GNSS+IMU)';
        $('speed-status-text').textContent = 'Signal GNSS verrouillé.';
    } else {
        $('gps-status-dr').textContent = 'Actif (Estimation Seule)';
        $('speed-status-text').textContent = 'Perte de signal GNSS. Estimation en cours...';
    }

    updateDisp(pos, dt);

    lPos = pos;
}

function handleErr(err) {
    if (wID !== null) {
        $('gps-status-dr').textContent = `ERREUR ${err.code}: ${err.message}`;
        $('speed-status-text').textContent = 'ERREUR GPS: Estimation active.';
    }
}

function toggleGPS() {
    const btn = $('toggle-gps-btn');
    const options = GPS_OPTS[$('freq-select').value];

    if (wID === null) {
        btn.innerHTML = '⏸️ PAUSE GPS';
        btn.style.backgroundColor = '#ffc107';
        wID = navigator.geolocation.watchPosition(handlePosition, handleErr, options);
        sTime = Date.now();
        $('gps-status-dr').textContent = 'MARCHE / Initialisation';
        $('speed-status-text').textContent = 'Acquisition des coordonnées...';
    } else {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        btn.innerHTML = '▶️ MARCHE GPS';
        btn.style.backgroundColor = '#28a745';
        $('gps-status-dr').textContent = 'PAUSE';
        $('speed-status-text').textContent = 'PAUSE. Données figées.';
        lastUpdateTime = 0;
    }
            }
// =========================================================================
// BLOC 4/4 : Démarrage Lent & Mise à Jour du DOM
// =========================================================================

function updateMap(lat, lon, acc) {
    if (typeof L === 'undefined') return;

    if (!map) {
        map = L.map('map').setView([lat, lon], 16);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);

        // Marqueur (Point rouge)
        marker = L.circleMarker([lat, lon], {
            radius: 6,
            color: 'white',
            fillColor: '#dc3545',
            fillOpacity: 1
        }).addTo(map);

        // Cercle de précision (Bleu transparent)
        accuracyCircle = L.circle([lat, lon], {
            radius: acc,
            color: '#007bff',
            fillColor: '#007bff',
            fillOpacity: 0.2,
            weight: 1
        }).addTo(map);

        $('map').textContent = ''; 
    }

    const newLatLng = L.latLng(lat, lon);
    marker.setLatLng(newLatLng);
    accuracyCircle.setLatLng(newLatLng);
    accuracyCircle.setRadius(acc);
    // map.setView(newLatLng); // Commenté pour éviter de recentrer constamment
    
    // Affichage de la précision basée sur l'estimation EKF
    $('gps-precision').textContent = `${currentEKFState.acc_est.toFixed(2)} m (Est.)`;
}

function updateDisp(pos, dt) {
    const V_ekf = getEKFVelocity3D();
    const sSpdFE = V_ekf < MIN_SPD ? 0 : V_ekf;
    const sSpdKMH = sSpdFE * KMH_MS;

    // --- 1. Vitesse et Distance ---
    maxSpd = Math.max(maxSpd, sSpdKMH);

    $('speed-stable').textContent = `${sSpdKMH.toFixed(3)} km/h`;
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
    $('speed-stable-kms').textContent = `${(sSpdFE * KMS_MS).toFixed(6)} km/s`;
    
    const rawSpeed = pos.coords.speed || 0;
    const rawSpeedKMH = rawSpeed * KMH_MS;
    $('speed-3d-inst').textContent = `${rawSpeedKMH.toFixed(3)} km/h`;
    $('speed-raw-ms').textContent = `${rawSpeed.toFixed(3)} m/s`;
    $('speed-max').textContent = `${maxSpd.toFixed(3)} km/h`;
    
    // Vitesse moyenne
    const totalTimeH = sTime ? (Date.now() - sTime) / 3600000 : 0;
    const totalTimeMovingH = timeMoving / 3600;
    
    if (totalTimeMovingH > 0) {
        $('speed-avg-moving').textContent = `${((distM_3D / 1000) / totalTimeMovingH).toFixed(3)} km/h`;
    }
    if (totalTimeH > 0) {
        $('speed-avg-total').textContent = `${((distM_3D / 1000) / totalTimeH).toFixed(3)} km/h`;
    }

    $('distance-total-km').textContent = `${(distM_3D / 1000).toFixed(3)} km | ${distM_3D.toFixed(2)} m`;
    $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    
    if (sTime !== null) {
        const elapsed = (Date.now() - sTime) / 1000;
        $('elapsed-time').textContent = `${elapsed.toFixed(2)} s`;
    }

    // --- 2. Dynamique et Forces ---
    const air_rho = lastAirDensity || 1.225; 
    
    // Vitesse du son & Mach
    const c_s = lastSpeedOfSound || C_S_BASE;
    const mach_number = sSpdFE / c_s;
    const perc_speed_sound = (sSpdFE / c_s) * 100;
    
    $('speed-of-sound-calc').textContent = `${c_s.toFixed(3)} m/s`;
    $('perc-speed-sound').textContent = `${perc_speed_sound.toFixed(2)} %`;
    $('mach-number').textContent = `${mach_number.toFixed(4)}`;

    // Pression Dynamique: q = 0.5 * rho * V^2
    const dynamic_pressure = 0.5 * air_rho * sSpdFE * sSpdFE;
    $('dynamic-pressure').textContent = `${dynamic_pressure.toFixed(2)} Pa`;
    
    // Force de Traînée: F_D = q * Cd * A
    const drag_force = dynamic_pressure * currentDragCoef * currentRefArea;
    $('drag-force').textContent = `${drag_force.toFixed(2)} N`;
    
    // Force de Coriolis
    const coriolis_force = 2 * currentMass * currentEKFState.V_e * OMEGA_EARTH * Math.cos(currentEKFState.lat * D2R);
    $('coriolis-force').textContent = `${coriolis_force.toExponential(1)} N`;
    
    // CORRECTION : Calcul de la gravité locale corrigée
    const currentAlt = currentEKFState.alt;
    let gravity_local_corr = G_ACC;
    if (currentAlt > 0) {
        const radiusRatio = R_ALT_CENTER_REF / (R_ALT_CENTER_REF + currentAlt);
        gravity_local_corr = G_ACC * radiusRatio * radiusRatio;
    }
    $('gravity-local').textContent = `${gravity_local_corr.toFixed(5)} m/s²`;
    
    lastAccelLong = real_accel_x; 
    $('accel-long').textContent = `${lastAccelLong.toFixed(3)} m/s²`;
    
    // Relativité
    const perc_speed_c = (sSpdFE / C_L) * 100;
    const lorentz_factor = 1 / Math.sqrt(1 - (sSpdFE*sSpdFE) / (C_L*C_L));
    $('perc-speed-c').textContent = `${perc_speed_c.toExponential(2)} %`;
    $('lorentz-factor').textContent = `${lorentz_factor.toFixed(4)}`;

    // --- 3. Position et EKF ---
    $('lat-display').textContent = `${currentEKFState.lat.toFixed(6)} °`;
    $('lon-display').textContent = `${currentEKFState.lon.toFixed(6)} °`;
    $('alt-display').textContent = `${currentEKFState.alt.toFixed(3)} m`;
    
    if (pos.coords.heading !== null && pos.coords.heading !== undefined) {
        $('heading-display').textContent = `${pos.coords.heading.toFixed(1)} °`;
    } 
    
    // Affichage de la précision GPS brute si disponible, sinon EKF
    if (pos.coords.accuracy !== null && pos.coords.accuracy !== undefined) {
        $('gps-accuracy-display').textContent = `${pos.coords.accuracy.toFixed(2)} m (Brut)`;
    } else {
        $('gps-accuracy-display').textContent = `${currentEKFState.acc_est.toFixed(2)} m (EKF Pos)`;
    }

    $('kalman-uncert').textContent = `${getVelocityUncertainty().toFixed(3)} m²/s² (P)`;
    const R_dynamic_disp = pos.coords.accuracy * pos.coords.accuracy * currentEnvFactor;
    $('speed-error-perc').textContent = `${R_dynamic_disp.toFixed(3)} m² (R dyn)`;
    
    // Mise à jour de la carte (fréquence réduite)
    if (pos.timestamp - lastMapUpdate > 3000) {
        updateMap(currentEKFState.lat, currentEKFState.lon, pos.coords.accuracy);
        lastMapUpdate = pos.timestamp;
    }
    
    highlightMissingData();
}

// --- Démarrage Lent (Initialisation, Météo, Astro) ---
async function startSlowLoop() {
    
    domID = setInterval(async () => {
        if (emergencyStopActive) {
            $('emergency-stop-btn').classList.add('active');
            highlightMissingData();
            return;
        }
        $('emergency-stop-btn').classList.remove('active');
        
        const currentLat = currentEKFState.lat;
        const currentLon = currentEKFState.lon;
        const nowMs = Date.now();

        // 1. Météo (Toutes les 10 secondes)
        if ((nowMs % 10000 < DOM_SLOW_UPDATE_MS * 2)) { // Utiliser un delta plus grand
            const data = await fetchWeather(currentLat, currentLon);
            
            if (data) {
                lastP_hPa = data.pressure_hPa; 
                lastT_K = data.tempK; 
                lastH_perc = data.humidity_perc;
                lastAirDensity = data.air_density;
                lastSpeedOfSound = data.speed_of_sound;
                
                if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                if ($('dew-point')) $('dew-point').textContent = `${data.dew_point} °C`;
                if ($('weather-status')) $('weather-status').textContent = data.status; 
            } else {
                if ($('weather-status')) $('weather-status').textContent = "HORS LIGNE MÉTÉO";
            }
        }

        // 2. Horloge locale (NTP) et UTC/GMT
        // lServH et lLocH sont maintenant initialisés au démarrage
        const timeSync = await syncH(lServH, lLocH);
        lServH = timeSync.lServH;
        lLocH = timeSync.lLocH;
        
        const now = getCDate(lServH, lLocH);
        const utc = now.toISOString().slice(0, 19).replace('T', ' ');
        
        $('local-time').textContent = now.toTimeString().slice(0, 8);
        $('date-display').textContent = `${utc} UTC/GMT`;
        $('time-minecraft').textContent = getMinecraftTime(now);

        // 3. Astronomie
        const astroData = updateAstro(currentLat, currentLon, lServH, lLocH);
        if (astroData) {
            const { sunPos, moonIllum, moonPos, solarTimes, skyStatus } = astroData;
            
            // Mise à jour de la Clock Minecraft (Rotation du soleil)
            const rotationAngle = (sunPos.azimuth * R2D) + 90; 
            $('sun-element').style.transform = `rotate(${rotationAngle}deg)`;
            $('minecraft-clock').className = ''; 
            $('minecraft-clock').classList.add(skyStatus);
            $('clock-status').textContent = `${skyStatus.replace('sky-', '')} (${(sunPos.altitude * R2D).toFixed(1)} °)`;

            // Mise à jour Astro
            $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
            $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D + 180).toFixed(2)} °`;
            $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
            $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
            $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
            $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D + 180).toFixed(2)} °`;

            $('tst').textContent = solarTimes.TST;
            $('mst').textContent = solarTimes.MST;
            $('eot').textContent = `${solarTimes.EOT} min`;
            $('ecl-long').textContent = `${solarTimes.ECL_LONG} °`;
            $('noon-solar').textContent = solarTimes.NoonSolar;
        }

        highlightMissingData();

    }, DOM_SLOW_UPDATE_MS);
}

// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
window.onload = () => {
    $('toggle-gps-btn').addEventListener('click', toggleGPS);
    
    // Arrêt d'Urgence
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = `🛑 Arrêt d'urgence: ${emergencyStopActive ? 'ACTIF' : 'INACTIF'} ${emergencyStopActive ? '🔴' : '🟢'}`;
        if (emergencyStopActive) {
            $('speed-status-text').textContent = 'ARRÊT D’URGENCE : Données figées.';
        }
    });
    
    // Mode Nuit/Jour
    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        $('toggle-mode-btn').innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
    });
    
    // Réinitialisations
    $('reset-all-btn').addEventListener('click', () => {
        if (confirm("Êtes-vous sûr de vouloir TOUT RÉINITIALISER ?")) {
            location.reload(); 
        }
    });
    $('reset-dist-btn').addEventListener('click', () => { distM_3D = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });


    // Contrôles EKF/Physique
    $('environment-select').addEventListener('change', (e) => {
        const factor = ENVIRONMENT_FACTORS[e.target.value];
        currentEnvFactor = factor.R_MULT;
        $('env-factor').textContent = `${factor.DISPLAY} (x${factor.R_MULT.toFixed(1)})`;
    });
    
    $('celestial-body-select').addEventListener('change', (e) => {
        const body = CELESTIAL_DATA[e.target.value];
        G_ACC = body.G;
        R_ALT_CENTER_REF = body.R;
        $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    });

    $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || DEFAULT_MASS;
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });

    $('gps-accuracy-override').addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
        // Met à jour le champ de lecture près de l'input
        $('gps-accuracy-display-input').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    });

    // --- INITIALISATION COMPLÈTE DU DOM AU CHARGEMENT ---
    
    // Initialisation du temps immédiate
    const now = new Date();
    const utc = now.toISOString().slice(0, 19).replace('T', ' ');
    $('local-time').textContent = now.toTimeString().slice(0, 8);
    $('date-display').textContent = `${utc} UTC/GMT`;
    
    // Initialisation EKF
    initEKF(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, DEFAULT_INIT_ALT, 10.0);
    $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    updateMap(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, 10.0);
    
    // Initialisation des champs EKF/Physique
    $('gravity-local').textContent = `${G_ACC.toFixed(5)} m/s²`;
    $('lat-display').textContent = `${currentEKFState.lat.toFixed(6)} °`;
    $('lon-display').textContent = `${currentEKFState.lon.toFixed(6)} °`;
    $('alt-display').textContent = `${currentEKFState.alt.toFixed(3)} m`;
    $('kalman-uncert').textContent = `${currentEKFState.P_vel.toFixed(3)} m²/s² (P)`;
    $('alt-uncertainty').textContent = `${currentEKFState.acc_est.toFixed(3)} m (EKF Pos)`;
    $('speed-error-perc').textContent = `${(R_MAX * R_MAX * 1.0).toFixed(3)} m² (R dyn)`;
    $('speed-of-sound-calc').textContent = `${C_S_BASE.toFixed(3)} m/s`;
    
    $('mass-display').textContent = `${DEFAULT_MASS.toFixed(3)} kg`;
    $('gps-accuracy-display').textContent = `N/A m (Est.)`; // Sera mis à jour par updateDisp
    
    startSlowLoop();
    
    // Tentative d'activation IMU sur mobile
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleIMU);
        $('imu-status').textContent = 'Actif (Accel)';
    }
}
