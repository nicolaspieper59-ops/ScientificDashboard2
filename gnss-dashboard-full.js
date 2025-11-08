// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes, Globales et Persistance)
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
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 

// !!! CLÉ API CRITIQUE !!! : REMPLACEZ VOTRE_CLE_API_OPENWEATHERMAP
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 

// Constantes EKF & ZVU (Zero Velocity Update)
const MIN_DT = 0.05; // Fréquence d'échantillonnage de l'EKF
const Q_NOISE = 0.001; // Bruit de processus (m/s²)
const MIN_SPD = 0.001; // Seuil de vitesse minimale (1 mm/s)

// Seuil plancher théorique pour l'incertitude d'état EKF (P)
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; // 5e-5 m² (implémentation du Plancher d'Incertitude EKF)

// --- VARIABLES GLOBALES (État du Système) ---
let lat = 0, lon = 0, alt = 0, speed = 0, hdop = 1000, gpsTS = 0;
let wID = null, domID = null, weatherID = null, emergencyStopActive = false;
let sTime = null, lastTS = 0, lastPos = null, timeMoving = 0;
let distM = 0; // Contient maintenant la distance 3D totale
let currentGPSMode = 'HIGH_FREQ'; 
let netherMode = false;
let G_ACC_LOCAL = G_ACC; 
let mass = 70.0; 
let lastAlt = 0; // Dernière altitude enregistrée pour le calcul 3D

// --- VARIABLES EKF (Filtre de Kalman Étendu) ---
let kSpd = 0;       
let kUncert = 1000; 
let kAlt = 0;       
let kAltUncert = 1000; 
let lastFSpeed = 0; 
let maxSpd = 0;     
let maxGForce = 0;  

// --- VARIABLES DE CAPTEURS IMU ---
let latestLinearAccelMagnitude = 0;
let latestAccelX = 0, latestAccelY = 0, latestAccelZ = 0;

// --- VARIABLES RECORDS DE PRÉCISION (Persistance) ---
let P_RECORDS = {
    max_kUncert_min: 1000, 
    max_acc_min: 1000,     
    max_g_force_max: 0     
};
const P_RECORDS_KEY = 'gnss_precision_records'; 

// --- UTILS ---
function getCDate() { return new Date(); }
function distanceCalc(lat1, lon1, lat2, lon2) {
    if (lat1 === 0 || lat2 === 0) return 0;
    const p = Math.PI / 180;
    const a = 0.5 - Math.cos((lat2 - lat1) * p)/2 + 
              Math.cos(lat1 * p) * Math.cos(lat2 * p) * (1 - Math.cos((lon2 - lon1) * p))/2;
    return 2 * R_E * Math.asin(Math.sqrt(a));
}
function getEnvironmentFactor() {
    const factorMap = { 'NORMAL': 1.0, 'FOREST': 1.5, 'CONCRETE': 3.0, 'METAL': 2.5 };
    const selected = $('environment-select').value;
    return factorMap[selected] || 1.0;
}

// ===========================================
// FONCTIONS PERSISTANCE DES RECORDS
// ===========================================
function loadPrecisionRecords() {
    try {
        const stored = localStorage.getItem(P_RECORDS_KEY);
        if (stored) {
            const loaded = JSON.parse(stored);
            P_RECORDS = { ...P_RECORDS, ...loaded };
            maxGForce = P_RECORDS.max_g_force_max;
        }
    } catch (e) {
        console.error("Erreur lors du chargement des records de précision:", e);
    }
}

function savePrecisionRecords() {
    try {
        P_RECORDS.max_g_force_max = maxGForce; 
        localStorage.setItem(P_RECORDS_KEY, JSON.stringify(P_RECORDS));
    } catch (e) {
        console.error("Erreur lors de la sauvegarde des records de précision:", e);
    }
            }
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
    speed = pos.coords.speed !== null ? pos.coords.speed : 0; // Vitesse horizontale brute du GPS
    gpsTS = pos.timestamp;
    
    const nowTS = getCDate().getTime();
    const dt = lastTS === 0 ? MIN_DT : (nowTS - lastTS) / 1000;
    lastTS = nowTS;

    // --- CALCUL VITESSE VERTICALE ET 3D ---
    let verticalSpeedRaw = 0;
    let speed3DInst = speed; 
    
    if (currentAlt !== null && lastAlt !== 0 && dt > 0) {
        verticalSpeedRaw = (currentAlt - lastAlt) / dt;
        // Vitesse 3D Instantannée = sqrt(V_horizontale_GPS² + V_verticale_GPS²)
        speed3DInst = Math.sqrt(speed * speed + verticalSpeedRaw * verticalSpeedRaw);
    }
    
    // --- EKF (Simplified 1D Speed/Position Filter) ---
    let kR = acc * acc * getEnvironmentFactor(); 
    let kGain = kUncert / (kUncert + kR);
    kSpd = kSpd + kGain * (speed - kSpd); // Filtre appliqué à la vitesse horizontale
    kUncert = (1 - kGain) * kUncert;
    kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); 

    // --- ACCÉLÉRATION & G-FORCE MAX ---
    let sSpdFE = Math.abs(kSpd);
    let accel_long = (dt > MIN_DT) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;

    const currentGForceLong = Math.abs(accel_long / G_ACC); 
    if (currentGForceLong > maxGForce) maxGForce = currentGForceLong; 

    // --- MISE À JOUR DOM ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(4)} km/h`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;

    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`; 
    
    $('speed-3d-inst').textContent = `${(speed3DInst * KMH_MS).toFixed(4)} km/h`; 
    $('vertical-speed').textContent = `${verticalSpeedRaw.toFixed(2)} m/s`;

    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G | Max: ${maxGForce.toFixed(2)} G`;

    // --- MISE À JOUR DES RECORDS DE PRÉCISION ---
    if (acc !== null && acc < P_RECORDS.max_acc_min) {
        P_RECORDS.max_acc_min = acc;
    }
    if (kUncert < P_RECORDS.max_kUncert_min) {
        P_RECORDS.max_kUncert_min = kUncert;
    }
    
    // Logique de distance et de mouvement (CALCUL 3D)
    if (lastPos) {
        const d_horiz = distanceCalc(lastPos.latitude, lastPos.longitude, lat, lon);
        
        let d_vert = 0;
        if (currentAlt !== null && lastAlt !== 0) {
            d_vert = Math.abs(currentAlt - lastAlt);
        }
        
        // Distance 3D = sqrt(d_horiz² + d_vert²)
        const d_3d = Math.sqrt(d_horiz * d_horiz + d_vert * d_vert); 
        distM += d_3d; 

        if (sSpdFE > MIN_SPD) {
            timeMoving += dt;
        }
    } 
    
    if (currentAlt !== null) lastAlt = currentAlt; 
    lastPos = { latitude: lat, longitude: lon };
    
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
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
    
    const totalGForce = latestLinearAccelMagnitude / G_ACC_LOCAL; 

    if (totalGForce > maxGForce) {
        maxGForce = totalGForce; 
    }
    
    $('accel-vertical-imu').textContent = `${(latestAccelZ - G_ACC_LOCAL).toFixed(3)} m/s²`;
    $('force-g-vertical').textContent = `${(latestAccelZ / G_ACC_LOCAL).toFixed(2)} G`;
}

/** Récupère les données météo via l'API OpenWeatherMap */
function getWeather() {
    if (OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = "API CLÉ MANQUANTE";
        return;
    }
    if (lat === 0 && lon === 0) return;

    fetch(`https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&units=metric&appid=${OWM_API_KEY}`)
        .then(response => response.json())
        .then(data => {
            if (data.main) {
                $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
                $('pressure').textContent = `${data.main.pressure} hPa`;
                $('humidity').textContent = `${data.main.humidity} %`;
            }
        })
        .catch(err => {
            console.error("Erreur de récupération Météo:", err);
            $('temp-air').textContent = "API ERREUR";
        });
        }
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
