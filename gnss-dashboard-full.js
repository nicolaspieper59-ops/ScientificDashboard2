// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes & Kalman)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES (Physiques, GPS, Temps) ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s)
const G_ACC_STD = 9.80665; // Gravité standard (m/s²)
const M_EARTH = 5.972e24; // Masse de la Terre (kg)
const G_CONST = 6.67430e-11; // Constante gravitationnelle (N(m/kg)²)
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const W_EARTH = 7.2921E-5; // Vitesse angulaire de la Terre (rad/s)
const NETHER_RATIO = 1 / 8; 

// Conversion de Vitesse
const KMH_MS = 3.6; 
const KMS_MS = 1000; // Conversion m/s -> km/s

// Constantes Cosmologiques pour la Distance
const AU_TO_M = 149597870700; // 1 Unité Astronomique en mètres
const LIGHT_YEAR_TO_M = 9.461e15; // 1 Année-Lumière en mètres

// Conversion de Temps-Lumière (en secondes)
const SEC_LIGHT = 1; 
const MIN_LIGHT = 60;
const HOUR_LIGHT = 3600;
const DAY_LIGHT = 86400;
const WEEK_LIGHT = 604800;
const MONTH_LIGHT = 2592000; // 30 jours (estimation)


// Constantes Temps / Astro
const dayMs = 86400000;
const J1970 = 2440588; 
const J2000 = 2451545; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// Constantes GPS
const MIN_DT = 0.05; 
const Q_NOISE = 0.001;
const MIN_SPD = 0.001; 
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; 

// --- VARIABLES GLOBALES (Records de Précision) ---
let P_RECORDS = {
    max_kUncert_min: 1000, 
    max_acc_min: 1000,     
    max_g_force_max: 0     
};
const P_RECORDS_KEY = 'gnss_precision_records'; 

// --- VARIABLES GLOBALES (EKF & État) ---
let lat = 0, lon = 0, alt = 0, speed = 0, gpsTS = 0;
let kSpd = 0, kUncert = 1000, kAlt = 0, kAltUncert = 1000;
let lastTS = 0, lastFSpeed = 0, distM = 0;
let lastPos = null, lastAlt = 0; 
let sTime = null, timeMoving = 0, maxSpd = 0; 
let latestAccelZ = 0, latestLinearAccelMagnitude = 0, maxGForce = 0;
let wID = null, domID = null, weatherID = null;
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let G_ACC_LOCAL = G_ACC_STD; // Gravité locale dynamique

// --- Clé API Météo ---
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 

// ===========================================
// FONCTIONS UTILITAIRES DE BASE
// ===========================================

/** Calcule la gravité locale en fonction de l'altitude. */
function calculateLocalGravity(altitude) {
    if (altitude === null) return G_ACC_STD;
    // Formule G(h) = G_CONST * M_EARTH / (R_E + h)^2
    const h = altitude;
    const g_local = G_CONST * M_EARTH / Math.pow(R_E + h, 2);
    // On garde la valeur locale pour le calcul des G-Forces
    G_ACC_LOCAL = g_local; 
    return g_local;
}

// ... (Gardez les fonctions loadPrecisionRecords, savePrecisionRecords, getCDate, getEnvironmentFactor, distanceCalc, toReadableScientific) ...
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js (EKF, Capteurs & Logique Critique)
// =================================================================

/** Simule la correction GPS théorique parfaite (Correction Théorique Automatique au Démarrage). */
function simulateBestCorrection() {
    if (lat === 0 && lon === 0) {
        kUncert = MIN_UNCERT_FLOOR; 
        kAltUncert = MIN_UNCERT_FLOOR; 
        return; 
    }

    const IDEAL_ACCURACY = 0.00001; 
    const mockBestCorrectionPos = {
        coords: { latitude: lat, longitude: lon, altitude: kAlt, accuracy: IDEAL_ACCURACY, speed: kSpd, altitudeAccuracy: IDEAL_ACCURACY },
        timestamp: new Date().getTime()
    };

    kUncert = MIN_UNCERT_FLOOR; 
    kAltUncert = MIN_UNCERT_FLOOR; 

    updateDisp(mockBestCorrectionPos); 
    
    console.log(`Simulateur de Correction Théorique activé : EKF réglé sur l'incertitude minimale (${MIN_UNCERT_FLOOR.toExponential(2)} m²).`);
}

/** Gestion du démarrage du GPS après l'autorisation IMU (iOS). */
function continueGPSStart() {
    const opts = { enableHighAccuracy: currentGPSMode === 'HIGH_FREQ', timeout: 5000, maximumAge: 0 };
    
    if (wID !== null) navigator.geolocation.clearWatch(wID);

    wID = navigator.geolocation.watchPosition(updateDisp, (error) => {
        console.warn(`ERREUR GPS(${error.code}): ${error.message}`);
        if (error.code === 1) {
            alert("Accès à la géolocalisation refusé. Le tableau de bord ne peut fonctionner.");
        }
    }, opts);

    if ($('freq-select')) $('freq-select').value = currentGPSMode; 
    sTime = sTime === null ? getCDate().getTime() : sTime; 
}

/** Démarre le GPS et gère l'autorisation des capteurs de mouvement (IMU/iOS). */
function startGPS() {
    if (wID === null) {
        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        continueGPSStart();
                    } else {
                        console.warn("Accès aux capteurs de mouvement refusé. Certaines données seront indisponibles.");
                        continueGPSStart(); 
                    }
                })
                .catch(err => {
                    console.error("Erreur d'autorisation DeviceMotion:", err);
                    continueGPSStart(); 
                });
        } else {
            continueGPSStart();
        }
    }
}

/** Handler des données GPS et du filtre EKF (Correction Automatique de Dérive GPS). */
function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    const acc = $('gps-accuracy-override').value !== '0.000000' ? parseFloat($('gps-accuracy-override').value) : pos.coords.accuracy;

    // --- LOGIQUE DE CORRECTION AUTOMATIQUE À LA FIN DE LA DÉRIVE (ZVU) ---
    const hasGPSFix = acc !== null && acc < 50; 
    const wasDrifting = kUncert > 1.0; 

    if (wasDrifting && hasGPSFix) {
        console.log("Correction automatique déclenchée : Signal GPS revenu après dérive.");
        lat = pos.coords.latitude; 
        lon = pos.coords.longitude;
        kAlt = pos.coords.altitude !== null ? pos.coords.altitude : kAlt; 
        kUncert = MIN_UNCERT_FLOOR; 
        kAltUncert = MIN_UNCERT_FLOOR; 
    }
    // ---------------------------------------------------

    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    const currentAlt = pos.coords.altitude; 
    alt = currentAlt; 
    const speedRaw = pos.coords.speed !== null ? pos.coords.speed : 0; 
    gpsTS = pos.timestamp;
    
    const currentHeading = pos.coords.heading !== null && !isNaN(pos.coords.heading) ? pos.coords.heading : 'N/A';
    
    const nowTS = getCDate().getTime();
    const dt = lastTS === 0 ? MIN_DT : (nowTS - lastTS) / 1000;
    lastTS = nowTS;

    // --- MISE À JOUR DE LA GRAVITÉ LOCALE (Gravité variable en fonction de l'altitude) ---
    const g_dynamic = calculateLocalGravity(currentAlt);
    $('gravity-local').textContent = `${g_dynamic.toFixed(5)} m/s²`;
    
    // --- EKF (Simplified 1D Speed/Position Filter) ---
    let kR = acc * acc * getEnvironmentFactor(); 
    let kGain = kUncert / (kUncert + kR);
    kSpd = kSpd + kGain * (speedRaw - kSpd); 
    kUncert = (1 - kGain) * kUncert;
    kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); 
    
    // --- ZVU ÉTALONNAGE (Correction de la dérive à l'arrêt) ---
    if (speedRaw < MIN_SPD * 10 && kSpd < MIN_SPD) { 
        kSpd = 0; 
    }
    
    // --- ACCÉLÉRATION & G-FORCE MAX (Fusion Hybride) ---
    let sSpdFE = Math.abs(kSpd);
    
    // Mise à jour de la Vitesse Max (CORRIGÉE: Utilise la vitesse EKF)
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const accel_ekf = (dt > MIN_DT) ? (sSpdFE - lastFSpeed) / dt : 0;
    
    let accel_imu_signed = 0;
    const IMU_ACCEL_THRESHOLD = 0.1; 
    const FUSION_FACTOR = 0.7; 

    if (latestLinearAccelMagnitude > IMU_ACCEL_THRESHOLD) { 
        accel_imu_signed = latestLinearAccelMagnitude * Math.sign(accel_ekf || 1); 
    }

    let accel_long;
    if (Math.abs(accel_imu_signed) > Math.abs(accel_ekf) && sSpdFE > 0.1) {
        accel_long = (accel_ekf * (1 - FUSION_FACTOR)) + (accel_imu_signed * FUSION_FACTOR);
    } else {
        accel_long = accel_ekf;
    }
    
    lastFSpeed = sSpdFE; 

    const currentGForceLong = Math.abs(accel_long / g_dynamic); 
    if (currentGForceLong > maxGForce) maxGForce = currentGForceLong; 

    // --- CALCUL VITESSE VERTICALE ET 3D (Utilise kSpd) ---
    let verticalSpeedRaw = 0;
    let speed3DInst = sSpdFE; 
    
    if (currentAlt !== null && lastAlt !== 0 && dt > 0) {
        verticalSpeedRaw = (currentAlt - lastAlt) / dt;
        speed3DInst = Math.sqrt(sSpdFE * sSpdFE + verticalSpeedRaw * verticalSpeedRaw);
    }
    
    // --- CALCUL DE LA DISTANCE (3D) ET VITESSE MOYENNE ---
    if (lastPos) {
        if (sSpdFE > MIN_SPD) { 
            const d_horiz = distanceCalc(lastPos.latitude, lastPos.longitude, lat, lon);
            
            let d_vert = 0;
            if (currentAlt !== null && lastAlt !== 0) {
                d_vert = Math.abs(currentAlt - lastAlt);
            }
            
            const d_3d = Math.sqrt(d_horiz * d_horiz + d_vert * d_vert); 
            distM += d_3d; 
            timeMoving += dt;
        }
    } 
    
    if (currentAlt !== null) lastAlt = currentAlt; 
    lastPos = { latitude: lat, longitude: lon };

    // --- MISE À JOUR DOM ---

    // Vitesse (km/h, km/s, m/s, nm/s)
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(4)} km/h`;
    $('speed-stable-kms').textContent = `${(sSpdFE / KMS_MS).toFixed(7)} km/s`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    
    // Vitesse Moyenne (CORRIGÉE)
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(4)} km/h`;

    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`; 
    
    $('speed-3d-inst').textContent = `${(speed3DInst * KMH_MS).toFixed(4)} km/h`; 
    $('vertical-speed').textContent = `${verticalSpeedRaw.toFixed(2)} m/s`;
    
    $('accel-long').textContent = `${(accel_long).toFixed(3)} m/s ²`; 
    $('force-g-long').textContent = `${(accel_long / g_dynamic).toFixed(2)} G | Max: ${maxGForce.toFixed(2)} G`;
    
    $('heading-display').textContent = currentHeading !== 'N/A' ? `${currentHeading.toFixed(1)} °` : 'N/A';

    // Mise à jour de la Distance 3D
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    // --- CALCULS DE DISTANCE COSMOLOGIQUE ---
    const distLightSeconds = distM / C_L;
    
    $('distance-light-s').textContent = `${toReadableScientific(distLightSeconds / SEC_LIGHT)} s lumière`;
    $('distance-light-min').textContent = `${toReadableScientific(distLightSeconds / MIN_LIGHT)} min lumière`;
    $('distance-light-h').textContent = `${toReadableScientific(distLightSeconds / HOUR_LIGHT)} h lumière`;
    $('distance-light-day').textContent = `${toReadableScientific(distLightSeconds / DAY_LIGHT)} j lumière`;
    $('distance-light-week').textContent = `${toReadableScientific(distLightSeconds / WEEK_LIGHT)} sem lumière`;
    $('distance-light-month').textContent = `${toReadableScientific(distLightSeconds / MONTH_LIGHT)} mois lumière`;
    
    // Affichage UA et Année-Lumière (al)
    const distAU = distM / AU_TO_M;
    const distLightYears = distM / LIGHT_YEAR_TO_M;
    $('distance-cosmic').textContent = `${toReadableScientific(distAU)} UA | ${toReadableScientific(distLightYears)} al`;
    
    // ... (Keep other DOM updates) ...
}

/** Handler pour les capteurs de mouvement (IMU) */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const accel = event.accelerationIncludingGravity;

    latestAccelZ = accel.z || 0;
    
    const linearAccel = event.acceleration;
    if (linearAccel) {
        latestLinearAccelMagnitude = Math.sqrt(linearAccel.x*linearAccel.x + linearAccel.y*linearAccel.y + linearAccel.z*linearAccel.z);
    } else {
        latestLinearAccelMagnitude = 0;
    }
    
    // Utilise la gravité locale dynamique pour le calcul de G-Force verticale
    const g_dynamic = G_ACC_LOCAL; 
    const totalGForce = latestLinearAccelMagnitude / g_dynamic; 

    if (totalGForce > maxGForce) {
        maxGForce = totalGForce; 
    }
    
    $('accel-vertical-imu').textContent = `${(latestAccelZ - g_dynamic).toFixed(3)} m/s²`;
    $('force-g-vertical').textContent = `${(latestAccelZ / g_dynamic).toFixed(2)} G`;
}

// ... (Gardez getWeather et les autres fonctions) ...
// =================================================================
// FICHIER JS PARTIE 3 : gnss-dashboard-part3.js (Astro, Événements & Initialisation)
// =================================================================

/** Calcule l'Heure Solaire Vraie (TST) en ms depuis le début du jour. */
function calculateTST(now, longitude) {
    const dayMs = 86400000;
    const msSinceMidnight = now.getHours() * 3600000 + now.getMinutes() * 60000 + now.getSeconds() * 1000 + now.getMilliseconds();
    
    // Obtenir le Temps Solaire Local (LST) pour l'heure actuelle
    // LST se base sur le Midi Solaire (Solar Noon)
    const ts = SunCalc.getTimes(now, 0, longitude); // On utilise 0 lat pour obtenir l'EOT pour la longitude
    const solarNoonUTC = ts.solarNoon;
    
    if (!solarNoonUTC) return 0; // Au-delà des cercles polaires

    // Différence entre le midi solaire réel et le midi civil (12:00:00)
    // Cette différence inclut l'Équation du Temps (EOT) et le décalage de longitude par rapport au fuseau horaire
    const noonOffsetMs = solarNoonUTC.getTime() - new Date(solarNoonUTC.getFullYear(), solarNoonUTC.getMonth(), solarNoonUTC.getDate(), 12, 0, 0).getTime();
    
    // TST = Temps Local - Décalage au Midi Solaire
    let tstMs = msSinceMidnight - noonOffsetMs;

    // Normalisation sur un cycle de 24h
    while (tstMs < 0) tstMs += dayMs;
    while (tstMs >= dayMs) tstMs -= dayMs;
    
    return tstMs;
}

/** Obtient le nom de la phase de la Lune à partir d'une valeur de phase [0, 1]. */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase >= 0.97) return 'Nouvelle Lune (🌑)';
    if (phase < 0.22) return 'Premier Croissant (🌒)';
    if (phase < 0.28) return 'Premier Quartier (🌓)';
    if (phase < 0.47) return 'Lune Gibbeuse Croissante (🌔)';
    if (phase < 0.53) return 'Pleine Lune (🌕)';
    if (phase < 0.72) return 'Lune Gibbeuse Décroissante (🌖)';
    if (phase < 0.78) return 'Dernier Quartier (🌗)';
    return 'Dernier Croissant (🌘)';
}

/** Anime l'horloge et change la couleur du ciel (inspiré par Minecraft). */
function updateMinecraftClock(sunAltitude, moonAltitude, tstMs) {
    const clock = $('minecraft-clock');
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const dayMs = 86400000;
    const R2D = 180 / Math.PI;
    
    // --- 1. LOGIQUE DE ROTATION (TST -> Angle) ---
    // La rotation simule l'arc : 
    // TST 00:00:00 -> Angle 0° (Lever EST)
    // TST 12:00:00 -> Angle 180° (Zénith SUD)
    // TST 24:00:00 -> Angle 360° (Coucher OUEST + retour à l'EST)
    const tstHours = tstMs / 3600000;
    const rotationDeg = (tstHours / 24) * 360; 
    
    // Rotation des icônes: elles sont positionnées au Zénith (top center) et le disque tourne.
    // Ajout d'un décalage initial de 90 degrés pour que 0h/24h soit à gauche (Lever/Coucher) et 12h soit au Zénith (haut)
    sunEl.style.transform = `rotate(${rotationDeg + 90}deg) translateY(-50%)`; 
    
    // La lune est à l'opposé du soleil (+180 degrés)
    const moonRotationDeg = rotationDeg + 90 + 180; 
    moonEl.style.transform = `rotate(${moonRotationDeg}deg) translateY(-50%)`;
    
    // --- 2. LOGIQUE COULEUR DU CIEL (sur le BODY) ---
    const altDeg = sunAltitude * R2D;
    let skyClass = '';
    
    if (altDeg > 15) { // Jour (Soleil bien haut)
        skyClass = 'sky-day';
    } else if (altDeg > 0) { // Crépuscule/Aube (0° à 15° - Horizon)
        skyClass = 'sky-sunset';
    } else if (altDeg > -12) { // Nuit nautique et civile (Nuit claire)
        skyClass = 'sky-night-light'; 
    } else { // Nuit astronomique (Nuit noire profonde)
        skyClass = 'sky-night';
    }

    // Appliquer la classe de couleur au BODY (si le Mode Nuit n'est pas forcé)
    document.body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'sky-night-light');
    if (!$('toggle-mode-btn').classList.contains('dark-mode')) {
         document.body.classList.add(skyClass);
    }
    
    // --- 3. LOGIQUE VISIBILITÉ ---
    const ALT_THRESHOLD_VIS = D2R * -6; // Seuil pour cacher/montrer (-6 degrés sous l'horizon)
    const sunIsVisible = sunAltitude > ALT_THRESHOLD_VIS;
    const moonIsVisible = moonAltitude > ALT_THRESHOLD_VIS;
    
    // Cacher si l'objet est trop loin sous l'horizon
    sunEl.style.opacity = sunIsVisible ? 1 : 0;
    moonEl.style.opacity = moonIsVisible ? 1 : 0;
    
    $('clock-status').textContent = `Progression du cycle: ${(tstMs / dayMs * 100).toFixed(1)}%`;
}


/** Mise à jour de l'affichage Astro et de l'animation. */
function updateAstro(latitude, longitude) {
    if (typeof SunCalc === 'undefined') return;
    const now = getCDate();
    const sunTimes = SunCalc.getTimes(now, latitude, longitude);
    const sunPos = SunCalc.getPosition(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latitude, longitude);
    const R2D = 180 / Math.PI;
    
    // 1. CALCUL TST
    const tstMs = calculateTST(now, longitude);
    const tstTime = new Date(tstMs).toISOString().substring(11, 19);
    
    // 2. MISE À JOUR DE L'ANIMATION ET DU CIEL
    updateMinecraftClock(sunPos.altitude, moonPos.altitude, tstMs);

    // 3. MISE À JOUR DU DOM (Statistiques)
    $('tst').textContent = tstTime;
    $('moon-phase-display').textContent = getMoonPhaseName(moonIllumination.phase); 
    $('sun-elevation').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
    
    $('noon-solar').textContent = sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';
    $('date-display-astro').textContent = now.toLocaleDateString();
    
    // Calcul EOT 
    if (sunTimes.solarNoon) {
        const EOT_ms = sunTimes.solarNoon.getTime() - new Date(sunTimes.solarNoon.getFullYear(), sunTimes.solarNoon.getMonth(), sunTimes.solarNoon.getDate(), 12, 0, 0).getTime();
        $('eot').textContent = `${(EOT_ms / 60000).toFixed(1)} min`;
    }
    
    $('ecliptic-long').textContent = `${(sunPos.azimuth * R2D).toFixed(1)} °`;
}

function syncH() { /* Non détaillé ici */ }

document.addEventListener('DOMContentLoaded', () => {
    // --- Initialisation du Système EKF & Persistence ---
    loadPrecisionRecords();
    simulateBestCorrection(); 

    // --- Événements GPS/Capteurs IMU ---
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté ou activé sur cet appareil/navigateur.");
    } 

    // --- Initialisation des intervalles ---
    syncH(); 
    startGPS(); 
    getWeather(); 

    // Intervalle lent pour les mises à jour DOM et Astro/Météo
    domID = setInterval(() => {
        const now = getCDate();
        if (now) {
            $('local-time').textContent = now.toLocaleTimeString();
            $('date-display').textContent = now.toLocaleDateString();
            $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime) / 1000).toFixed(2) + ' s' : '0.00 s';
            $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
        }
        if (lat !== 0 && lon !== 0) updateAstro(lat, lon); 
    }, DOM_SLOW_UPDATE_MS); 
    
    weatherID = setInterval(getWeather, WEATHER_UPDATE_MS);

    // --- Gestion des Boutons ---
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { 
        if (wID) {
            navigator.geolocation.clearWatch(wID);
            wID = null;
            $('toggle-gps-btn').textContent = " 7œ4„1‚5 MARCHE GPS";
        } else {
            startGPS();
            $('toggle-gps-btn').textContent = " 7œ4„1‚5 ARRÊT GPS";
        }
    });

    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', (e) => {
        e.currentTarget.classList.toggle('dark-mode'); // Utilisé pour forcer le Dark Mode
        document.body.classList.toggle('dark-mode');
        // S'assurer que les classes de ciel sont retirées si le mode nuit est activé manuellement
        if (document.body.classList.contains('dark-mode')) {
             document.body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'sky-night-light');
        }
    });

    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? "•0“5 Arrêt d'urgence: ACTIF •0 4" : "•0“5 Arrêt d'urgence: INACTIF •0 4";
    });

    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser? (Distance, Max, Kalman)")) { 
            distM = 0; maxSpd = 0; maxGForce = 0; 
            kSpd = 0; kUncert = MIN_UNCERT_FLOOR; kAlt = 0; kAltUncert = MIN_UNCERT_FLOOR; timeMoving = 0; lastFSpeed = 0;
            savePrecisionRecords(); 
        } 
    });

    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => {
        if (emergencyStopActive) return; 
        maxSpd = 0;
        maxGForce = 0;
        P_RECORDS.max_g_force_max = 0;
        savePrecisionRecords(); 
        alert("Vitesse Max et G-Force Max réinitialisées.");
    });
    
    window.addEventListener('beforeunload', savePrecisionRecords);
});
