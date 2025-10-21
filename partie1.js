// ===========================================
// Fichier JavaScript - partie1.js
// Contient: Constantes, Variables Globales, Fonctions de Calcul (GNSS, Astro, Kalman), Fonctions utilitaires (Temps, Cible).
// ===========================================

// ===========================================
// 1. CONSTANTES ET VARIABLES GLOBALES
// ===========================================

// --- PHYSIQUE / VITESSES ---
const C_LIGHT = 299792458;          
const C_SOUND_SEA_LEVEL = 343;      
const EARTH_RADIUS = 6371000;       
const KMH_PER_MS = 3.6;             
const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

// --- CONSTANTES ASTRO ---
const OBLIQUITY = 23.44 * DEG_TO_RAD; 
const ECCENTRICITY = 0.0167;        
const JD_2000_REF = 2451545.0; 
let timeOffsetMS = 0; 

// --- ÉTAT DE L'APPLICATION ---
const WATCH_OPTIONS = {
    enableHighAccuracy: true,
    maximumAge: 100,
    timeout: 5000 
};
const DOM_UPDATE_INTERVAL_MS = 17;
let watchID = null;         
let domIntervalID = null;
let lastPosition = null;
let startTime = null;
let totalDistanceM = 0; 
let maxSpeedMS = 0;
let targetLat = null;
let targetLon = null;
let lastDOMTime = null;

// --- FILTRAGE GPS & SENSITIVITÉ ---
const MIN_TIME_INTERVAL_S = 1; 
const MAX_ACCURACY_M = 50;     
const MIN_SPEED_THRESHOLD_MS = 0.001;
const UNDERGROUND_ALT_THRESHOLD_M = -50; 

// --- FILTRAGE DE KALMAN ADAPTATIF ---
let kalmanSpeed = 0; 
let kalmanUncertainty = 1000; 
const PROCESS_NOISE_Q = 0.005; 
const KALMAN_R_MIN = 0.005;     
const KALMAN_R_MAX = 5.0;       
const LOW_PRECISION_THRESHOLD_M = 60;   


// --- REFERENCES DOM (Partielles - Références pleines dans la partie 2) ---
const modeIndicator = document.getElementById('mode-indicator');
const setTargetBtn = document.getElementById('set-target-btn');


// ===========================================
// 2. SYNCHRONISATION DU TEMPS
// ===========================================

async function synchronizeTime() {
    const timeServerUrl = 'https://worldtimeapi.org/api/ip'; 
    try {
        const response = await fetch(timeServerUrl);
        const T4 = Date.now(); 
        const data = await response.json();
        const serverTimeUTC = data.utc_datetime;
        const serverTimeMS = new Date(serverTimeUTC).getTime();
        timeOffsetMS = serverTimeMS - T4;
    } catch (e) {
        timeOffsetMS = 0; 
    }
}

function getCorrectedDate() {
    return new Date(Date.now() + timeOffsetMS);
}


// ===========================================
// 3. FONCTIONS DE CALCULS GÉO ET ASTRO
// ===========================================

function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = EARTH_RADIUS;
    const dLat = (lat2 - lat1) * DEG_TO_RAD;
    const dLon = (lon2 - lon1) * DEG_TO_RAD;
    lat1 *= DEG_TO_RAD;
    lat2 *= DEG_TO_RAD;

    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1) * Math.cos(lat2) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    const R = DEG_TO_RAD;
    lat1 *= R; lon1 *= R; lat2 *= R; lon2 *= R;

    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) -
              Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    
    let bearing = Math.atan2(y, x) * RAD_TO_DEG;
    return (bearing + 360) % 360; 
}

function calculateSolarDetails() {
    const now = getCorrectedDate();
    const J2000 = new Date(Date.UTC(2000, 0, 1, 12, 0, 0));
    const D = (now.getTime() - J2000.getTime()) / (1000 * 60 * 60 * 24);

    const M = (357.529 + 0.98560028 * D) * DEG_TO_RAD;
    const L = (280.466 + 0.98564736 * D) * DEG_TO_RAD;
    
    const C_e = 2 * ECCENTRICITY * Math.sin(M) + 1.25 * ECCENTRICITY * ECCENTRICITY * Math.sin(2 * M);
    const lambda = L + C_e; 

    const alpha = Math.atan2(Math.cos(OBLIQUITY) * Math.sin(lambda), Math.cos(lambda));
    
    const EoT_rad = L - alpha;
    const EoT_minutes = (EoT_rad * RAD_TO_DEG) * 4;
    
    return {
        eot: EoT_minutes,
        solarLongitude: (lambda * RAD_TO_DEG) % 360,
    };
}

function calculateLunarPhaseAngle() {
    const now = getCorrectedDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const d = JD - JD_2000_REF; 
    
    let D = 297.8501921 + 445.2671115 * d;

    D = D % 360; 
    if (D < 0) { D += 360; }
    
    return D * DEG_TO_RAD;
}

function calculateLunarTime(longitude) {
    const now = getCorrectedDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 

    const T = (JD - JD_2000_REF) / 36525.0; 
    let GST = 280.4606 + 360.9856473 * (JD - JD_2000_REF) + 0.000388 * T * T;
    GST = GST % 360;
    if (GST < 0) GST += 360;

    let L_moon = 218.316 + 488204.661 * T; 
    L_moon = L_moon % 360;
    if (L_moon < 0) L_moon += 360;
    
    let HA_moon = GST + longitude - L_moon;
    HA_moon = HA_moon % 360;
    if (HA_moon < 0) HA_moon += 360;

    const LMT_hours = HA_moon / 15.0; 
    
    const LMT_total_seconds = LMT_hours * 3600;
    const lmtHour = Math.floor(LMT_total_seconds / 3600);
    const lmtMinute = Math.floor((LMT_total_seconds % 3600) / 60);
    const lmtSecond = Math.floor(LMT_total_seconds % 60);
    
    document.getElementById('lunar-time').textContent = 
        `${String(lmtHour).padStart(2, '0')}:${String(lmtMinute).padStart(2, '0')}:${String(lmtSecond).padStart(2, '0')}`;
}

// ===========================================
// 4. FILTRAGE DE KALMAN ADAPTATIF
// ===========================================

function simpleKalmanFilter(newSpeedMS, dt, R_dynamic) {
    if (dt === 0 || dt > 5) { return kalmanSpeed; } 
    
    const R = R_dynamic !== undefined ? R_dynamic : KALMAN_R_MAX; 
    const Q = PROCESS_NOISE_Q * dt; 
    
    let predictedSpeed = kalmanSpeed;
    let predictedUncertainty = kalmanUncertainty + Q; 

    let K = predictedUncertainty / (predictedUncertainty + R); 

    kalmanSpeed = predictedSpeed + K * (newSpeedMS - predictedSpeed);
    kalmanUncertainty = (1 - K) * predictedUncertainty;

    return kalmanSpeed;
}


// ===========================================
// 5. GESTION DU MODE JOUR/NUIT & CIBLE
// ===========================================

function updateDarkMode(latitude, longitude) {
    const now = getCorrectedDate();
    const localHourApprox = now.getUTCHours() + now.getUTCMinutes() / 60 + longitude / 15;
    const hours = localHourApprox % 24;
    
    const SUNRISE_H = 6;
    const SUNSET_H = 18;

    const isNight = (hours < SUNRISE_H || hours >= SUNSET_H);

    if (isNight) {
        document.body.classList.add('night-mode');
        modeIndicator.textContent = 'Mode: Nuit 🌑';
    } else {
        document.body.classList.remove('night-mode');
        modeIndicator.textContent = 'Mode: Jour ☀️';
    }
}

function setTargetDestination() {
    if (!lastPosition) {
        alert("Veuillez démarrer le GPS et attendre une position avant de définir une cible.");
        return;
    }

    const currentLat = lastPosition.coords.latitude.toFixed(6);
    const currentLon = lastPosition.coords.longitude.toFixed(6);
    
    const inputLat = prompt(`Entrez la Latitude de destination (actuel: ${currentLat}°) :`, currentLat);
    const inputLon = prompt(`Entrez la Longitude de destination (actuel: ${currentLon}°) :`, currentLon);

    const lat = parseFloat(inputLat);
    const lon = parseFloat(inputLon);

    if (!isNaN(lat) && !isNaN(lon)) {
        targetLat = lat;
        targetLon = lon;
        setTargetBtn.textContent = '✅ Cible définie';
    } else {
        alert("Coordonnées invalides. Réinitialisation.");
        targetLat = null;
        targetLon = null;
        document.getElementById('cap-dest').textContent = 'N/A';
        setTargetBtn.textContent = '📍 Aller';
    }
}

function updateAstroDisplay(latitude, longitude) {
    const now = getCorrectedDate();
    
    // --- TEMPS MINECRAFT ---
    const mcTicksPerDay = 24000;
    const msSinceMidnight = now.getTime() % 86400000;
    const mcTicks = (msSinceMidnight * mcTicksPerDay) / 86400000;
    const mcMinutes = Math.floor(mcTicks / 20) % 1440; 
    const mcHour = Math.floor(mcMinutes / 60);
    const mcMinute = mcMinutes % 60;
    const mcSecond = Math.floor((mcTicks % 20) / 20 * 60);
    document.getElementById('mc-time').textContent = `${String(mcHour).padStart(2, '0')}:${String(mcMinute).padStart(2, '0')}:${String(mcSecond).padStart(2, '0')}`;


    // --- TEMPS SOLAIRE MOYEN (HSM) & HSV ---
    const totalSecondsUT = now.getUTCHours() * 3600 + now.getUTCMminutes() * 60 + now.getUTCSeconds();
    const totalSecondsLSM = (totalSecondsUT + (longitude * 4 * 60) + 86400) % 86400;
    
    const hsmHour = Math.floor(totalSecondsLSM / 3600);
    const hsmMinute = Math.floor((totalSecondsLSM % 3600) / 60);
    const hsmSecond = Math.floor(totalSecondsLSM % 60);
    
    document.getElementById('solar-mean').textContent = 
        `${String(hsmHour).padStart(2, '0')}:${String(hsmMinute).padStart(2, '0')}:${String(hsmSecond).padStart(2, '0')}`;
    
    const solarDetails = calculateSolarDetails();
    const EoT_seconds = solarDetails.eot * 60;
    
    const totalSecondsLSM_Corrected = totalSecondsLSM + EoT_seconds; 
    const totalSecondsHSV = (totalSecondsLSM_Corrected + 86400) % 86400;

    const lsvHour = Math.floor(totalSecondsHSV / 3600); 
    const lsvMinute = Math.floor((totalSecondsHSV % 3600) / 60);
    const lsvSecond = Math.floor(totalSecondsHSV % 60);
    
    const hsvTimeStr = `${String(lsvHour).padStart(2, '0')}:${String(lsvMinute).padStart(2, '0')}:${String(lsvSecond).padStart(2, '0')}`;

    document.getElementById('solar-true').textContent = `${hsvTimeStr} (HSV)`;
    document.getElementById('solar-true-header').textContent = hsvTimeStr;

    document.getElementById('eot').textContent = `${solarDetails.eot.toFixed(2)} min`;
    document.getElementById('solar-longitude-val').textContent = `${solarDetails.solarLongitude.toFixed(2)} °`;


    // --- PHASE LUNAIRE (Illumination) ---
    const D_rad = calculateLunarPhaseAngle();
    const illumination = 0.5 * (1 - Math.cos(D_rad)); 
    document.getElementById('lunar-phase-perc').textContent = `${(illumination * 100).toFixed(1)}%`;

    // --- TEMPS LUNAIRE ---
    calculateLunarTime(longitude); 
}
// Fin du fichier partie1.js (Coupure artificielle à la ligne 500 pour la copie)
