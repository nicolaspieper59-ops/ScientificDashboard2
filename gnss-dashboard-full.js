// =========================================================================
// BLOC 1/4 : Constantes, État Global & Initialisation
// =========================================================================

// CORRECTION : Import de la fonction météo réelle du module client
import { fetchWeatherReal } from './weather.js';
// Les librairies SunCalc et Leaflet sont chargées globalement via index.html


// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; // Degrés vers Radians / Radians vers Degrés
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Facteur de conversion m/s -> km/h
const KMS_MS = 1 / 1000;    
const C_S_BASE = 340.29;    // Vitesse du son standard (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse angulaire de la Terre (rad/s)
const R_MAX = 500.0;        // Précision GPS maximale acceptée

// --- PARAMÈTRES DU FILTRE DE KALMAN (EKF) ---
const Q_NOISE = 0.5;        // Bruit de processus (m/s)
const MIN_SPD = 0.05;       // Vitesse minimale pour être considéré en mouvement (m/s)
const MIN_DT = 0.01;        // Delta Temps minimum (pour éviter la division par zéro)
const ZUPT_ACCEL_TOLERANCE = 0.5; // Tolérance d'accélération pour ZUPT (m/s²)

// --- PARAMÈTRES DYNAMIQUES PAR DÉFAUT ---
const DEFAULT_MASS = 70.0; 
const DEFAULT_DRAG_COEF = 0.5; 
const DEFAULT_REF_AREA = 1.0; 

// --- CONSTANTES DE L'APPLICATION ---
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 250; // Fréquence de rafraîchissement des données lentes
const WEATHER_UPDATE_PERIOD = 10000; // Fréquence de l'appel météo (10 secondes)
const DEFAULT_INIT_LAT = 50.063237; 
const DEFAULT_INIT_LON = 14.445111;
const DEFAULT_INIT_ALT = 264.20;

// --- STRUCTURES DE DONNÉES DE RÉFÉRENCE ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: 6371000, DISPLAY: 'Terre' },
    'MARS': { G: 3.72076, R: 3389500, DISPLAY: 'Mars' },
    'MOON': { G: 1.625, R: 1737400, DISPLAY: 'Lune' },
};

const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- ÉTAT DU FILTRE EKF (Variables de l'état estimé) ---
let currentEKFState = {
    lat: DEFAULT_INIT_LAT, lon: DEFAULT_INIT_LON, alt: DEFAULT_INIT_ALT,
    V_n: 0.0, V_e: 0.0, V_d: 0.0, // Vitesse Nord, Est, Bas (Down)
    acc_est: 10.0, // Incertitude de position estimée (m)
    P_pos: 100.0, P_vel: 0.625 // Covariance de position et vitesse
};

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, domID = null, lPos = null, sTime = null; // ID Watcher GPS, ID Intervalle DOM, Dernière Position GPS, Heure de début session
let distM_3D = 0, maxSpd = 0, timeMoving = 0, lastUpdateTime = 0; // Distance totale (m), Vitesse max (km/h), Temps de mouvement (s)
let lServH = Date.now(), lLocH = Date.now(); // Simulation de l'heure serveur (pour NTP)
let emergencyStopActive = false;
let currentMass = DEFAULT_MASS; 
let currentDragCoef = DEFAULT_DRAG_COEF, currentRefArea = DEFAULT_REF_AREA;
let angularVelocity = 0.0, gpsAccuracyOverride = 0.0; 
let G_ACC = CELESTIAL_DATA['EARTH'].G; R_ALT_CENTER_REF = R_E_BASE;            
let currentEnvFactor = 1.0;
let lastP_hPa = null, lastT_K = null, lastH_perc = null, lastAirDensity = null; // Données atmosphériques stockées
let lastSpeedOfSound = C_S_BASE;
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0; // Données IMU
let lastAccelLong = 0;
let map, marker, accuracyCircle; // Objets Leaflet
let lastMapUpdate = 0;


// --- UTILITIES ---
const $ = id => document.getElementById(id);

// Fonction de gestion de l'heure de référence (simule NTP)
function getCDate(lServH, lLocH) {
    if (lServH === null || lLocH === null) {
        return new Date();
    }
    const offset = lServH - lLocH;
    return new Date(Date.now() + offset);
}

/**
 * Met en évidence les champs de données manquantes.
 */
function highlightMissingData() {
    const missingTexts = ['N/A', '--', 'INACTIF', 'ERREUR']; 
    const valueSpans = document.querySelectorAll('.data-point span:last-child');
    
    let quoteToUse;
    if (emergencyStopActive) {
        quoteToUse = "« Arrêt d'urgence. Session figée. »";
    } else if (wID === null) {
        quoteToUse = "« Mode Hors-Ligne: Sauvegarde active. »";
    } else {
        quoteToUse = "« Données en attente. »";
    }

    valueSpans.forEach(span => {
        const currentText = span.textContent.trim().toUpperCase();
        let isMissing = missingTexts.some(mt => currentText.includes(mt));
        
        if (span.id.includes('emergency-stop')) return;
        
        if (/[0-9]/.test(currentText) && (currentText.includes('M/S') || currentText.includes('KG/M³') || currentText.includes('PA') || currentText.includes('N') || currentText.includes('°'))) {
             isMissing = false; 
        }
        
        if (isMissing) {
            span.classList.add('value-missing');
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

/**
 * Simule la synchronisation du temps (NTP) pour compenser le décalage local.
 */
async function syncH(lServH, lLocH) {
    // Dans une application réelle, on ferait une requête AJAX ici.
    return new new Promise((resolve) => {
        const localTimeBefore = Date.now();
        const serverTime = Date.now() + 150; // Simule un serveur avec 150ms d'avance
        const localTimeAfter = Date.now();
        const latency = (localTimeAfter - localTimeBefore) / 2;
        
        const newServH = serverTime + latency;
        const newLocH = localTimeAfter;
        
        resolve({ lServH: newServH, lLocH: newLocH });
    });
}

// L'appel à fetchWeatherReal est géré par l'importation au début du fichier.
async function fetchWeather(lat, lon) {
    return fetchWeatherReal(lat, lon);
}

/**
 * Calcule l'heure Minecraft (pour la visualisation fun).
 */
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

/**
 * Calcule les données astronomiques (Soleil, Lune, TST, EOT) en utilisant SunCalc.
 */
function updateAstro(lat, lon, lServH, lLocH) {
    if (typeof SunCalc === 'undefined' || lat === 0 || lon === 0) return null;
    
    const now = getCDate(lServH, lLocH); 
    const daySinceJ2000 = (now.getTime() - new Date(2000, 0, 1, 12, 0, 0).getTime()) / 86400000;
    
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    
    // Calcul de l'Équation du Temps (EOT) et TST
    const M_deg = (357.5291 + 0.98560028 * daySinceJ2000); 
    const M_rad_trig = M_deg * D2R; 
    
    const L_deg = M_deg + (1.9148 * Math.sin(M_rad_trig)) + (0.0200 * Math.sin(2 * M_rad_trig)) + 102.9372;
    let L_rad = (L_deg % 360) * D2R; 
    
    const E_rad = (23.4393 - 3.563e-7 * daySinceJ2000) * D2R;
    const RA_rad = Math.atan2(Math.cos(E_rad) * Math.sin(L_rad), Math.cos(L_rad));
    
    let EOT_deg_diff = (L_deg - RA_rad * R2D); 
    EOT_deg_diff = EOT_deg_diff % 360;
    if (EOT_deg_diff > 180) EOT_deg_diff -= 360;
    if (EOT_deg_diff < -180) EOT_deg_diff += 360;
    
    const EOT_min = EOT_deg_diff * 4; 

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
    let skyStatus = 'night';
    const alt_deg = sunPos.altitude * R2D;
    
    if (alt_deg > 5) {
        skyStatus = 'day'; 
    } else if (alt_deg > -0.833) {
        skyStatus = 'sunset'; 
    } else if (alt_deg > -18) {
        skyStatus = 'night-light'; 
    } else {
        skyStatus = 'night'; 
    }

    return {
        now, sunPos, moonIllum, moonPos,
        solarTimes: {
            TST: decimalToTime(TST_hours), MST: decimalToTime(MST_hours),
            EOT: EOT_min.toFixed(2), ECL_LONG: (L_deg % 360).toFixed(2), 
            NoonSolar: decimalToTime(noon_utc_h)
        },
        skyStatus
    };
}

/**
 * Renvoie le nom de la phase de la lune.
 */
function getMoonPhaseName(phase) {
    if (phase < 0.06 || phase > 0.94) return "Nouvelle Lune";
    if (phase < 0.25) return "Premier Croissant";
    if (phase < 0.44) return "Premier Quartier";
    if (phase < 0.56) return "Pleine Lune";
    if (phase < 0.75) return "Gibbeuse Décroissante";
    if (phase < 0.94) return "Dernier Quartier";
    return "Phase Inconnue";
}


// --- CORE LOGIC EKF (Simplified) ---

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

/**
 * Étape de prédiction de l'EKF (Dead Reckoning + IMU).
 */
function predictEKF(dt, acc_imu, G_ACC, R_ALT_CENTER_REF) {
    const Vn = currentEKFState.V_n;
    const Ve = currentEKFState.V_e;
    const lat_rad = currentEKFState.lat * D2R; 

    const accel_n = acc_imu[0]; // Accélération Nord
    const accel_e = acc_imu[1]; // Accélération Est
    const accel_d = acc_imu[2]; // Accélération Bas

    // 1. Prédiction de la Vitesse (V = V0 + a*dt)
    currentEKFState.V_n += accel_n * dt;
    currentEKFState.V_e += accel_e * dt;
    currentEKFState.V_d += accel_d * dt;

    const R_MERIDIAN = R_ALT_CENTER_REF; 
    const R_TRANSVERSE = R_ALT_CENTER_REF * Math.cos(lat_rad); 
    
    // 2. Prédiction de la Position (P = P0 + V*dt)
    currentEKFState.lat += (Vn * dt) / R_MERIDIAN * R2D; 
    currentEKFState.lon += (Ve * dt) / R_TRANSVERSE * R2D; 
    currentEKFState.alt -= currentEKFState.V_d * dt; 

    // 3. Prédiction de l'Incertitude (P)
    currentEKFState.P_pos += Q_NOISE * dt * 0.1; 
    currentEKFState.P_vel += Q_NOISE * dt; 
}

/**
 * Étape de correction de l'EKF (Correction GNSS).
 */
function updateEKF_GNSS(gnss_pos, gnss_vel, accRaw, altAcc) {
    const R_dynamic = accRaw * accRaw * currentEnvFactor; // Bruit de mesure dynamique (R)
    
    // Correction de Position (Lat/Lon)
    const K_pos = currentEKFState.P_pos / (currentEKFState.P_pos + R_dynamic); // Gain de Kalman
    
    currentEKFState.lat += K_pos * (gnss_pos.lat - currentEKFState.lat);
    currentEKFState.lon += K_pos * (gnss_pos.lon - currentEKFState.lon);
    
    // Correction d'Altitude
    const K_alt = currentEKFState.P_pos / (currentEKFState.P_pos + altAcc * altAcc);
    currentEKFState.alt += K_alt * (gnss_pos.alt - currentEKFState.alt);

    // Mise à jour de la Covariance (P)
    currentEKFState.P_pos = (1 - K_pos) * currentEKFState.P_pos;
    currentEKFState.acc_est = Math.sqrt(currentEKFState.P_pos); 

    // Correction de Vitesse
    if (gnss_vel.V_n !== null && gnss_vel.V_n !== undefined) {
        const R_vel = R_dynamic * 0.1; 
        const K_vel = currentEKFState.P_vel / (currentEKFState.P_vel + R_vel);
        
        currentEKFState.V_n += K_vel * (gnss_vel.V_n - currentEKFState.V_n);
        currentEKFState.V_e += K_vel * (gnss_vel.V_e - currentEKFState.V_e); 
        currentEKFState.V_d += K_vel * (gnss_vel.V_d - currentEKFState.V_d); 

        currentEKFState.P_vel = (1 - K_vel) * currentEKFState.P_vel;
    }
}

/**
 * Mise à jour de vitesse zéro (ZUPT): Réinitialise la vitesse quand le mouvement est faible.
 */
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
// BLOC 3/4 : Gestion des Événements IMU & GPS (Watchers)
// =========================================================================

function handleIMU(event) {
    if (event.acceleration) {
        // Stockage des accélérations brutes (non compensées par la gravité)
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
    // Calcul du Delta Temps (dt)
    const dt = lastUpdateTime === 0 ? MIN_DT : Math.max(MIN_DT, (cTimePos - lastUpdateTime) / 1000); 
    lastUpdateTime = cTimePos;
    
    if (emergencyStopActive) {
        updateDisp(pos, dt);
        return;
    }

    // --- MISE À JOUR EKF (PRÉDICTION) ---
    const accel_imu = [real_accel_x, real_accel_y, real_accel_z]; 
    predictEKF(dt, accel_imu, G_ACC, R_ALT_CENTER_REF);

    if (lPos === null) {
        // Initialisation à la première position GPS
        initEKF(pos.coords.latitude, pos.coords.longitude, pos.coords.altitude || DEFAULT_INIT_ALT, pos.coords.accuracy);
        sTime = Date.now();
    } else {
        const dist_ekf = getEKFVelocity3D() * dt;
        distM_3D += dist_ekf;

        // --- ZUPT (Zero Update): Correction à vitesse zéro ---
        const currentAccelMagnitude = Math.sqrt(real_accel_x*real_accel_x + real_accel_y*real_accel_y + real_accel_z*real_accel_z);
        if (getEKFVelocity3D() < MIN_SPD && currentAccelMagnitude < ZUPT_ACCEL_TOLERANCE) {
            updateEKF_ZUPT();
        }

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
        
        if (pos.coords.heading !== null && pos.coords.heading !== undefined) {
            const headingRad = pos.coords.heading * D2R;
            V_gnss_n = rawSpeed * Math.cos(headingRad);
            V_gnss_e = rawSpeed * Math.sin(headingRad);
        } else {
            // Approximation si le cap n'est pas disponible (simple vitesse lat/lon)
            V_gnss_n = rawSpeed;
            V_gnss_e = 0.0; 
        }

        const gnss_vel = { V_n: V_gnss_n, V_e: V_gnss_e, V_d: 0.0 }; 
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
    const options = GPS_OPTS['HIGH_FREQ']; // Utilise HIGH_FREQ par défaut

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
// BLOC 4/4 : Affichage DOM & Boucles Lentes (Render)
// =========================================================================

/**
 * Met à jour les éléments visuels de l'horloge et de l'état du ciel.
 */
function updateClockVisualization(now, sunPos, moonPos, sunTimes, skyStatus) {
    const sunEl = $('sun-element');
    const clockEl = $('minecraft-clock'); 

    if (!sunEl || !clockEl || !sunPos || sunPos.azimuth === undefined) return;
    
    // Rotation du Soleil (Azimut)
    const rotationAngle = (sunPos.azimuth * R2D) + 90; 
    sunEl.style.transform = `rotate(${rotationAngle}deg)`;
    
    // Couleur du Ciel
    clockEl.className = ''; 
    clockEl.classList.add(`sky-${skyStatus}`);
    
    $('clock-status').textContent = `${skyStatus.toUpperCase()} (${(sunPos.altitude * R2D).toFixed(1)}°)`;
}

/**
 * Met à jour la carte Leaflet.
 */
function updateMap(lat, lon, acc) {
    if (typeof L === 'undefined') {
        $('map').textContent = "Erreur: Librairie Leaflet non chargée.";
        return;
    }

    if (!map) {
        map = L.map('map').setView([lat, lon], 16);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);

        marker = L.circleMarker([lat, lon], {
            radius: 6, color: 'white', fillColor: '#dc3545', fillOpacity: 1
        }).addTo(map);

        accuracyCircle = L.circle([lat, lon], {
            radius: acc, color: '#007bff', fillColor: '#007bff', fillOpacity: 0.2, weight: 1
        }).addTo(map);

        $('map').textContent = ''; 
    }

    const newLatLng = L.latLng(lat, lon);
    marker.setLatLng(newLatLng);
    accuracyCircle.setLatLng(newLatLng);
    accuracyCircle.setRadius(acc);
    
    // Recentrer la carte toutes les 3 secondes
    if (Date.now() - lastMapUpdate > 3000) {
        map.setView(newLatLng);
        lastMapUpdate = Date.now();
    }
    
    $('gps-precision').textContent = `${currentEKFState.acc_est.toFixed(2)} m (Est.)`;
}

/**
 * Met à jour l'ensemble des valeurs d'affichage rapide (Vitesse, EKF, Dynamique).
 */
function updateDisp(pos, dt) {
    const V_ekf = getEKFVelocity3D();
    const sSpdFE = V_ekf < MIN_SPD ? 0 : V_ekf; // Vitesse filtrée EKF
    const sSpdKMH = sSpdFE * KMH_MS;

    // --- 1. Vitesse et Distance ---
    maxSpd = Math.max(maxSpd, sSpdKMH);

    $('speed-stable').textContent = `${sSpdKMH.toFixed(3)} km/h`;
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
    
    const rawSpeed = pos.coords.speed || 0;
    const rawSpeedKMH = rawSpeed * KMH_MS;
    $('speed-3d-inst').textContent = `${rawSpeedKMH.toFixed(3)} km/h`;
    $('speed-max').textContent = `${maxSpd.toFixed(3)} km/h`;
    
    const totalTimeH = sTime ? (Date.now() - sTime) / 3600000 : 0;
    const totalTimeMovingH = timeMoving / 3600;
    
    if (totalTimeMovingH > 0) {
        $('speed-avg-moving').textContent = `${((distM_3D / 1000) / totalTimeMovingH).toFixed(3)} km/h`;
    }
    
    $('distance-total-km').textContent = `${(distM_3D / 1000).toFixed(3)} km | ${distM_3D.toFixed(2)} m`;
    $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    
    if (sTime !== null) {
        const elapsed = (Date.now() - sTime) / 1000;
        $('elapsed-time').textContent = `${elapsed.toFixed(2)} s`;
    }

    // --- 2. Dynamique et Forces ---
    const air_rho = lastAirDensity || 1.225; 
    const c_s = lastSpeedOfSound || C_S_BASE;
    const mach_number = sSpdFE / c_s;
    
    $('speed-of-sound-calc').textContent = `${c_s.toFixed(3)} m/s`;
    $('perc-speed-sound').textContent = `${(sSpdFE / c_s * 100).toFixed(2)} %`;
    $('mach-number').textContent = `${mach_number.toFixed(4)}`;

    const dynamic_pressure = 0.5 * air_rho * sSpdFE * sSpdFE;
    $('dynamic-pressure').textContent = `${dynamic_pressure.toFixed(2)} Pa`;
    
    const drag_force = dynamic_pressure * currentDragCoef * currentRefArea;
    $('drag-force').textContent = `${drag_force.toFixed(2)} N`;
    
    // Calcul de la gravité locale (correction d'altitude)
    let gravity_local_corr = G_ACC;
    const currentAlt = currentEKFState.alt;
    if (currentAlt > 0) {
        const radiusRatio = R_ALT_CENTER_REF / (R_ALT_CENTER_REF + currentAlt);
        gravity_local_corr = G_ACC * radiusRatio * radiusRatio;
    }
    $('gravity-local').textContent = `${gravity_local_corr.toFixed(5)} m/s²`;
    
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
    
    if (pos.coords.accuracy !== null && pos.coords.accuracy !== undefined) {
        $('gps-accuracy-display').textContent = `${pos.coords.accuracy.toFixed(2)} m (Brut)`;
    }
    
    $('kalman-uncert').textContent = `${getVelocityUncertainty().toFixed(3)} m²/s² (P)`;
    const R_dynamic_disp = pos.coords.accuracy * pos.coords.accuracy * currentEnvFactor;
    $('speed-error-perc').textContent = `${R_dynamic_disp.toFixed(3)} m² (R dyn)`;
    
    updateMap(currentEKFState.lat, currentEKFState.lon, pos.coords.accuracy);
    
    highlightMissingData();
}

/**
 * Boucle lente pour les mises à jour à faible fréquence (Astro, Météo, Temps).
 */
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
        if ((nowMs % WEATHER_UPDATE_PERIOD < DOM_SLOW_UPDATE_MS * 2)) {
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
                
                // Calcul de l'altitude barométrique
                if ($('alt-baro') && lastP_hPa) {
                     const baroAlt = 44330 * (1.0 - Math.pow(lastP_hPa / 1013.25, 0.1903));
                     $('alt-baro').textContent = `${baroAlt.toFixed(0)} m`;
                }
            } else {
                if ($('weather-status')) $('weather-status').textContent = "HORS LIGNE MÉTÉO";
            }
        }

        // 2. Horloge locale (NTP) et UTC/GMT
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
            const { now, sunPos, moonIllum, moonPos, solarTimes, skyStatus } = astroData;
            
            updateClockVisualization(now, sunPos, moonPos, null, skyStatus); 
            
            $('date-astro').textContent = now.toLocaleDateString();
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
    
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = `🛑 Arrêt d'urgence: ${emergencyStopActive ? 'ACTIF' : 'INACTIF'} ${emergencyStopActive ? '🔴' : '🟢'}`;
    });
    
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').innerHTML = isDarkMode ? 'Mode Jour' : 'Mode Nuit';
        });
    }

    // Gestion des boutons de réinitialisation
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { location.reload(); });
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM_3D = 0; timeMoving = 0; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });

    // Contrôles EKF/Physique (Select et Input)
    if ($('environment-select')) {
        $('environment-select').addEventListener('change', (e) => {
            const factor = ENVIRONMENT_FACTORS[e.target.value];
            currentEnvFactor = factor.R_MULT;
            $('env-factor').textContent = `${factor.DISPLAY} (x${factor.R_MULT.toFixed(1)})`;
        });
    }
    
    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => {
            const body = CELESTIAL_DATA[e.target.value];
            G_ACC = body.G;
            R_ALT_CENTER_REF = body.R;
            $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        });
    }

    if ($('mass-input')) {
        $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || DEFAULT_MASS;
            $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
    }

    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('input', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
            if ($('gps-accuracy-display-input')) { 
                 $('gps-accuracy-display-input').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
            }
        });
    }

    // --- INITIALISATION DE L'ÉTAT INITIAL DU DOM ---
    initEKF(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, DEFAULT_INIT_ALT, 10.0);
    if ($('map')) updateMap(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, 10.0);
    updateDisp({ coords: { speed: 0, accuracy: 10.0, heading: 0, latitude: DEFAULT_INIT_LAT, longitude: DEFAULT_INIT_LON } }, MIN_DT);

    startSlowLoop();
    
    // Logique d'activation IMU (nécessite un clic sur iOS/Safari)
    function requestIMUPermission() {
        if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        window.addEventListener('devicemotion', handleIMU);
                        if ($('imu-status')) $('imu-status').textContent = 'Actif (Accel)';
                    } else {
                        if ($('imu-status')) $('imu-status').textContent = 'Permission IMU refusée';
                    }
                })
                .catch(() => { if ($('imu-status')) $('imu-status').textContent = 'Erreur permission IMU'; });
        } else if (window.DeviceMotionEvent) {
            window.addEventListener('devicemotion', handleIMU);
            if ($('imu-status')) $('imu-status').textContent = 'Actif (Accel)';
        } else {
            if ($('imu-status')) $('imu-status').textContent = 'IMU non supporté';
        }
    }

    document.body.addEventListener('click', requestIMUPermission, { once: true });
    requestIMUPermission(); 
}
