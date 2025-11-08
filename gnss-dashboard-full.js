// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes, Gravité & Persistence)
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

// --- CONSTANTES PLANÉTAIRES (Gravité de surface et masses/rayons) ---
const G_SURFACE_MOON = 1.625; // Gravité de surface de la Lune (m/s²)
const R_MOON = 1737400; // Rayon moyen de la Lune (m)
const M_MOON = 7.34767e22; // Masse de la Lune (kg)

const G_SURFACE_MARS = 3.721; // Gravité de surface de Mars (m/s²)
const R_MARS = 3389500; // Rayon moyen de Mars (m)
const M_MARS = 6.4171e23; // Masse de Mars (kg)

// Constantes Temps / Astro
const dayMs = 86400000;
const J1970 = 2440588; 
const J2000 = 2451545; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// Constantes GPS
const MIN_DT = 0.05; 
const Q_NOISE = 0.001; // Bruit du processus (modèle)
const MIN_SPD = 0.001; // 1 mm/s (Seuil de vitesse pour ZVU)
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; // Plancher d'incertitude EKF

// --- VARIABLES GLOBALES (Records de Précision & État) ---
let P_RECORDS = {
    max_kUncert_min: 1000, 
    max_acc_min: 1000,     
    max_g_force_max: 0     
};
const P_RECORDS_KEY = 'gnss_precision_records'; 

let lat = 0, lon = 0, alt = 0, speed = 0, gpsTS = 0;
let kSpd = 0, kUncert = 1000, kAlt = 0, kAltUncert = 1000;
let lastTS = 0, lastFSpeed = 0, distM = 0;
let lastPos = null, lastAlt = 0; 
let sTime = null, timeMoving = 0, maxSpd = 0; 
let latestAccelZ = 0, latestLinearAccelMagnitude = 0, maxGForce = 0;
let wID = null, domID = null, weatherID = null;
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let currentCelestialBody = 'EARTH'; 
let G_ACC_LOCAL = G_ACC_STD; // Gravité locale dynamique

// --- Clé API Météo (À REMPLACER) ---
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 

// ===========================================
// FONCTIONS UTILITAIRES ET GRAVITÉ DYNAMIQUE
// ===========================================

/** Calcule la gravité locale en fonction de l'altitude et du corps céleste. */
function calculateLocalGravity(altitude) {
    let R, M;
    let g_surface;

    // Détermine les constantes basées sur le corps sélectionné
    switch (currentCelestialBody) {
        case 'MOON':
            R = R_MOON; M = M_MOON; g_surface = G_SURFACE_MOON;
            break;
        case 'MARS':
            R = R_MARS; M = M_MARS; g_surface = G_SURFACE_MARS;
            break;
        case 'EARTH':
        default:
            R = R_E; M = M_EARTH; g_surface = G_ACC_STD;
            break;
    }
    
    // Si nous n'avons pas d'altitude GPS ou si nous ne sommes pas sur Terre, 
    // ou si l'altitude est nulle (Rover/Exploration), on retourne la gravité de surface.
    if (altitude === null || isNaN(altitude) || currentCelestialBody !== 'EARTH') {
        G_ACC_LOCAL = g_surface; 
        return g_surface; 
    }
    
    // Formule G(h) = G_CONST * M / (R + h)^2 (Calcul d'altitude uniquement pour la Terre)
    const h = altitude;
    const g_local = G_CONST * M / Math.pow(R + h, 2);
    
    G_ACC_LOCAL = g_local; 
    return g_local;
}

/** Met à jour le corps céleste actuel et recalcule la gravité. */
function updateCelestialBody(body) {
    currentCelestialBody = body;
    console.log(`Corps céleste réglé sur : ${body}`);
    // Recalcule la gravité immédiatement en utilisant l'altitude connue (alt)
    calculateLocalGravity(alt); 
    // Mise à jour de l'affichage
    $('gravity-local').textContent = `${G_ACC_LOCAL.toFixed(5)} m/s²`;
}

/** Charge les records de précision depuis localStorage au démarrage. */
function loadPrecisionRecords() {
    try {
        const stored = localStorage.getItem(P_RECORDS_KEY);
        if (stored) {
            const loaded = JSON.parse(stored);
            P_RECORDS = { ...P_RECORDS, ...loaded };
            maxGForce = P_RECORDS.max_g_force_max;
        }
    } catch (e) {
        console.error("Erreur de chargement des records:", e);
    }
}

/** Sauvegarde les records de précision dans localStorage. */
function savePrecisionRecords() {
    P_RECORDS.max_g_force_max = maxGForce; 
    try {
        localStorage.setItem(P_RECORDS_KEY, JSON.stringify(P_RECORDS));
    } catch (e) {
        console.error("Erreur de sauvegarde des records:", e);
    }
}

/** Obtient l'heure actuelle (NTP sync si possible, sinon locale). */
function getCDate() {
    return sTime ? new Date(new Date().getTime() - (sTime.getTime() - gpsTS)) : new Date();
}

/** Calcule la distance entre deux coordonnées (Formule Haversine simplifiée pour petites distances). */
function distanceCalc(lat1, lon1, lat2, lon2) {
    if (lat1 === 0 && lon1 === 0) return 0;
    const R = R_E; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

/** Obtient le facteur de bruit R pour l'EKF en fonction de l'environnement sélectionné. */
function getEnvironmentFactor() {
    const env = $('environment-select').value;
    switch (env) {
        case 'FOREST': return 1.5;
        case 'METAL': return 2.5;
        case 'CONCRETE': return 3.0; 
        default: return 1.0; 
    }
}

/** Formate un nombre en notation scientifique lisible (ex: 1.23 x 10^4). */
function toReadableScientific(num) {
    if (num === 0) return "0.00";
    const exponent = Math.floor(Math.log10(Math.abs(num)));
    const mantissa = num / Math.pow(10, exponent);
    return `${mantissa.toFixed(2)}e+${exponent}`;
}
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js (EKF, ZVU, Fusion & Logique 3D/Cosmique)
// =================================================================

/** Simule la correction GPS théorique parfaite (Correction Théorique Automatique au Démarrage). */
function simulateBestCorrection() {
    if (lat === 0 && lon === 0) {
        kUncert = MIN_UNCERT_FLOOR; 
        kAltUncert = MIN_UNCERT_FLOOR; 
        return; 
    }

    // Réinitialise l'EKF à l'incertitude minimale connue
    kUncert = MIN_UNCERT_FLOOR; 
    kAltUncert = MIN_UNCERT_FLOOR; 
    
    // Force une mise à jour pour appliquer le nouveau statut
    const mockBestCorrectionPos = {
        coords: { latitude: lat, longitude: lon, altitude: kAlt, accuracy: 0.00001, speed: kSpd, altitudeAccuracy: 0.00001 },
        timestamp: new Date().getTime()
    };
    updateDisp(mockBestCorrectionPos); 
    
    console.log(`Simulateur de Correction Théorique activé : EKF réglé sur l'incertitude minimale (${MIN_UNCERT_FLOOR.toExponential(2)} m²).`);
}

/** Handler des données GPS et du filtre EKF (Correction Automatique de Dérive GPS). */
function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    const acc = $('gps-accuracy-override').value !== '0.000000' ? parseFloat($('gps-accuracy-override').value) : pos.coords.accuracy;

    // --- LOGIQUE DE CORRECTION AUTOMATIQUE À LA FIN DE LA DÉRIVE ---
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

    // --- MISE À JOUR DE LA GRAVITÉ LOCALE (Gravité dynamique) ---
    const g_dynamic = calculateLocalGravity(currentAlt);
    $('gravity-local').textContent = `${g_dynamic.toFixed(5)} m/s²`;
    
    // --- EKF (Simplified 1D Speed/Position Filter) ---
    let kR = acc * acc * getEnvironmentFactor(); 
    let kGain = kUncert / (kUncert + kR);
    kSpd = kSpd + kGain * (speedRaw - kSpd); 
    kUncert = (1 - kGain) * kUncert;
    kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); // Plancher d'incertitude
    
    // --- ZVU ÉTALONNAGE (Correction de la dérive à l'arrêt) ---
    if (speedRaw < MIN_SPD * 10 && kSpd < MIN_SPD) { 
        kSpd = 0; 
    }
    
    // --- ACCÉLÉRATION & G-FORCE MAX (Fusion Hybride IMU/EKF) ---
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

    // --- CALCUL VITESSE VERTICALE ET 3D ---
    let verticalSpeedRaw = 0;
    let speed3DInst = sSpdFE; 
    
    if (currentAlt !== null && lastAlt !== 0 && dt > 0) {
        verticalSpeedRaw = (currentAlt - lastAlt) / dt;
        // Vitesse 3D = sqrt(V_EKF_Horiz^2 + V_Vertical_Raw^2)
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
            
            const d_3d = Math.sqrt(d_horiz * d_horiz + d_vert * d_vert); // Distance 3D
            distM += d_3d; 
            timeMoving += dt;
        }
    } 
    
    if (currentAlt !== null) lastAlt = currentAlt; 
    lastPos = { latitude: lat, longitude: lon };

    // --- MISE À JOUR DOM : VITESSE ---

    // Vitesse (km/h, km/s, m/s, nm/s)
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(4)} km/h`;
    $('speed-stable-kms').textContent = `${(sSpdFE / KMS_MS).toFixed(7)} km/s`; // km/s
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    
    // Vitesse Moyenne (CORRIGÉE)
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(4)} km/h`;

    // Vitesse ultra-précise (nm/s)
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`; 
    
    $('speed-3d-inst').textContent = `${(speed3DInst * KMH_MS).toFixed(4)} km/h`; 
    $('vertical-speed').textContent = `${verticalSpeedRaw.toFixed(2)} m/s`;
    
    // Pourcentages Lumière/Son
    $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)}%`;
    $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)}%`;

    // --- MISE À JOUR DOM : ACCÉLÉRATION & G-FORCE ---
    $('accel-long').textContent = `${(accel_long).toFixed(3)} m/s ²`; 
    $('force-g-long').textContent = `${(accel_long / g_dynamic).toFixed(2)} G | Max: ${maxGForce.toFixed(2)} G`;
    
    // --- MISE À JOUR DOM : DISTANCE (3D & Cosmologique) ---
    
    // Distance 3D
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    // CALCULS DE DISTANCE COSMOLOGIQUE
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
    
    // --- MISE À JOUR DOM : GPS BRUT ET PHYSIQUE ---
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = currentAlt !== null ? `${currentAlt.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = acc !== null ? `${acc.toFixed(3)} m` : 'N/A';
    $('speed-raw-ms').textContent = speedRaw !== null ? `${speedRaw.toFixed(3)} m/s` : 'N/A';
    $('heading-display').textContent = currentHeading !== 'N/A' ? `${currentHeading.toFixed(1)} °` : 'N/A';

    // Sous-sol
    $('underground-status').textContent = currentAlt !== null && currentAlt < -50 ? 'OUI' : 'Non';

    // Énergie Cinétique (basée sur la vitesse EKF)
    const mass = parseFloat($('mass-display').textContent) || 70;
    $('kinetic-energy').textContent = `${(0.5 * mass * sSpdFE * sSpdFE).toFixed(2)} J`;
    $('mechanical-power').textContent = `${(mass * accel_long * sSpdFE).toFixed(2)} W`;

    // --- Mise à jour des records de précision ---
    if (acc !== null && acc < P_RECORDS.max_acc_min) P_RECORDS.max_acc_min = acc;
    if (kUncert < P_RECORDS.max_kUncert_min) P_RECORDS.max_kUncert_min = kUncert;
    savePrecisionRecords();

    // Mise à jour de la carte (implémentée dans part3.js)
    if (lat !== 0 && lon !== 0) updateMap(lat, lon); 
                 }
// =================================================================
// FICHIER JS PARTIE 3 : gnss-dashboard-part3.js (Astro, UI, Listeners & Init)
// =================================================================

let map = null;
let marker = null;

// --- FONCTIONS ASTRO / TST ---

/** Calcule et affiche l'heure solaire vraie (TST) et l'animation astro. */
function updateAstro(latitude, longitude) {
    if (latitude === 0 && longitude === 0) return;

    const now = getCDate();
    const times = SunCalc.getTimes(now, latitude, longitude);
    const pos = SunCalc.getPosition(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latitude, longitude);

    const solarNoon = times.solarNoon.getTime();
    const dayEnd = times.night.getTime();
    const dayStart = times.sunrise.getTime();

    // 1. Calcul de l'Heure Solaire Vraie (TST)
    let fractionalDay = (now.getTime() - times.solarMidnight.getTime()) / dayMs;
    let tstHours = fractionalDay * 24;
    tstHours = tstHours % 24;

    const h = Math.floor(tstHours);
    const m = Math.floor((tstHours - h) * 60);
    const s = Math.floor(((tstHours - h) * 60 - m) * 60);

    $('tst').textContent = `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    $('time-minecraft').textContent = $('tst').textContent; 

    // 2. Animation Jour/Nuit (Rotation et Couleur de fond)
    const sunAzimuthDeg = (pos.azimuth + Math.PI) * R2D; // 0=Nord, 90=Est, 180=Sud, 270=Ouest
    const sunAltDeg = pos.altitude * R2D;
    
    // Rotation TST: L'horloge est centrée sur le TST. Midi Solaire (Zénith) est à 180 degrés de rotation.
    let clockRotation = (tstHours / 24) * 360; 
    
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');

    // On utilise la position azimutale pour la rotation de l'horloge
    sunEl.style.transform = `rotate(${clockRotation - 90}deg)`; 
    moonEl.style.transform = `rotate(${(moonPos.azimuth + Math.PI) * R2D - 90}deg)`; 

    // 3. Changement de couleur de fond (Style Minecraft-like)
    let skyClass = '';
    const elevationThreshold = 0; 
    const isNight = sunAltDeg < elevationThreshold;

    if (!isNight) {
        if (sunAltDeg > 20) {
            skyClass = 'sky-day'; 
        } else {
            skyClass = 'sky-sunset'; 
        }
    } else {
        if (sunAltDeg > -12) { 
            skyClass = 'sky-night-light'; 
        } else {
            skyClass = 'sky-night'; 
        }
    }

    // Applique la classe au body, sauf si le mode nuit est activé
    if (!$('toggle-mode-btn').classList.contains('active')) {
        document.body.className = skyClass;
    }


    // 4. Affichages Astro supplémentaires
    $('sun-elevation').textContent = `${sunAltDeg.toFixed(2)} °`;
    $('noon-solar').textContent = times.solarNoon.toLocaleTimeString();
    $('day-duration').textContent = `${((dayEnd - dayStart) / 3600000).toFixed(2)} h`;
    $('moon-phase-display').textContent = `${(moonIllumination.phase * 100).toFixed(1)}% | ${getMoonPhaseName(moonIllumination.phase)}`;
    $('ecliptic-long').textContent = `${(pos.eclipticLng * R2D).toFixed(2)} °`;
    
    // Calcul de l'Équation du Temps (EOT)
    const meanSolarTime = (now.getTime() - times.solarMidnight.getTime()) / dayMs * 24;
    const tstSec = h * 3600 + m * 60 + s;
    const lstSec = (meanSolarTime % 24) * 3600;
    const eotMin = (tstSec - lstSec) / 60;
    $('eot').textContent = `${eotMin.toFixed(2)} min`;
}

/** Nomme la phase lunaire. */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune";
    if (phase < 0.22) return "Premier Croissant";
    if (phase < 0.28) return "Premier Quartier";
    if (phase < 0.47) return "Lune Gibbeuse Croissante";
    if (phase < 0.53) return "Pleine Lune";
    if (phase < 0.72) return "Lune Gibbeuse Décroissante";
    if (phase < 0.78) return "Dernier Quartier";
    return "Dernier Croissant";
}

// --- LOGIQUE GPS ET DÉMARRAGE ---

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
    sTime = sTime === null ? getCDate() : sTime; 
}

function startGPS() {
    if (wID === null) {
        // Demande d'autorisation pour les capteurs de mouvement (IMU/iOS)
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
    $('toggle-gps-btn').textContent = ' 7œ4„1‚5 ARRÊT GPS';
    $('toggle-gps-btn').style.backgroundColor = '#dc3545';
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    $('toggle-gps-btn').textContent = ' 7œ4„1‚5 MARCHE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#28a745';
}

// --- HANDLERS CAPTEURS IMU ---

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const accel = event.accelerationIncludingGravity;

    latestAccelZ = accel.z || 0; // Accélération verticale avec gravité
    
    const linearAccel = event.acceleration;
    if (linearAccel) {
        // Magnitude de l'accélération linéaire totale (sans gravité)
        latestLinearAccelMagnitude = Math.sqrt(linearAccel.x*linearAccel.x + linearAccel.y*linearAccel.y + linearAccel.z*linearAccel.z);
    } else {
        latestLinearAccelMagnitude = 0;
    }
    
    // Utilise la gravité locale dynamique pour le calcul de G-Force verticale
    const g_dynamic = G_ACC_LOCAL; 
    const totalGForce = latestLinearAccelMagnitude / g_dynamic; 

    if (totalGForce > maxGForce) maxGForce = totalGForce; 
    
    $('accel-vertical-imu').textContent = `${(latestAccelZ - g_dynamic).toFixed(3)} m/s²`;
    $('force-g-vertical').textContent = `${(latestAccelZ / g_dynamic).toFixed(2)} G`;

    // Également mis à jour dans updateDisp pour la persistence
    savePrecisionRecords(); 
}

// --- SYNCHRONISATION HORLOGE (NTP) ---

function syncH() {
    fetch(SERVER_TIME_ENDPOINT)
        .then(response => response.json())
        .then(data => {
            const apiTime = new Date(data.utc_datetime).getTime();
            const localTime = new Date().getTime();
            gpsTS = apiTime + (localTime - apiTime); 
            sTime = new Date();
            console.log("Synchronisation de l'heure (NTP) réussie.");
        })
        .catch(err => {
            console.warn("Échec de la synchronisation NTP, utilise l'heure locale.", err);
            sTime = new Date();
        });
}

// --- MÉTÉO (FAUSSE API EN ATTENTE DE REMPLACEMENT) ---

function getWeather() {
    // Remplacer VOTRE_CLE_API_OPENWEATHERMAP par une clé réelle
    if (lat === 0 || lon === 0 || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API Requis';
        // ... (mettez tous les champs météo sur 'API Requis' si besoin)
        return;
    }
    
    const url = `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${OWM_API_KEY}&units=metric`;

    fetch(url)
        .then(response => response.json())
        .then(data => {
            if (data.main) {
                $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
                $('pressure').textContent = `${data.main.pressure.toFixed(0)} hPa`;
                $('humidity').textContent = `${data.main.humidity}%`;
                $('wind-speed-ms').textContent = `${data.wind.speed.toFixed(1)} m/s`;
                // ... (mettre à jour les autres champs météo)
            }
        })
        .catch(err => console.error("Erreur de récupération météo :", err));
}

// --- CARTE LEAFLET ---

function initMap() {
    map = L.map('map-container').setView([0, 0], 2);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);

    marker = L.marker([0, 0]).addTo(map)
        .bindPopup("Position Actuelle")
        .openPopup();
}

function updateMap(latitude, longitude) {
    if (map) {
        map.setView([latitude, longitude], map.getZoom() < 12 ? 12 : map.getZoom());
        marker.setLatLng([latitude, longitude]);
    }
}

// --- INITIALISATION FINALE DU SYSTÈME ET LISTENERS ---

document.addEventListener('DOMContentLoaded', () => {
    loadPrecisionRecords();
    initMap();
    syncH(); 
    
    // Initialisation IMU
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté ou activé sur cet appareil/navigateur.");
    } 

    // Initialisation du corps céleste
    const initialAlt = alt; // alt est global, initialement 0
    updateCelestialBody($('celestial-body-select').value);
    
    // --- Intervalle de mise à jour lente DOM/Astro ---
    domID = setInterval(() => {
        // Mise à jour de l'heure et de la date locales
        const now = getCDate();
        if (now) {
            $('local-time').textContent = now.toLocaleTimeString();
            $('date-display').textContent = now.toLocaleDateString();
            $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime.getTime()) / 1000).toFixed(2) + ' s' : '0.00 s';
            $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
        }
        
        // Mise à jour Astro si on a une position
        if (lat !== 0 && lon !== 0) updateAstro(lat, lon); 
    }, DOM_SLOW_UPDATE_MS); 
    
    // --- Intervalle pour la mise à jour Météo (30s) ---
    weatherID = setInterval(getWeather, WEATHER_UPDATE_MS); 

    // --- Événements Utilisateur ---
    $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS() : stopGPS());
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '•0“5 Arrêt d\'urgence: ACTIF •0 4' : '•0“5 Arrêt d\'urgence: INACTIF •0 4';
        $('emergency-stop-btn').style.backgroundColor = emergencyStopActive ? '#f8d7da' : '#dc3545';
        if (emergencyStopActive) stopGPS();
    });
    $('nether-toggle-btn').addEventListener('click', () => {
        // Toggle Nether Mode (pas entièrement implémenté dans la logique EKF, mais le ratio est prêt)
        const isActive = $('mode-nether').textContent.includes('ACTIF');
        $('mode-nether').textContent = isActive ? 'DÉSACTIVÉ (1:1)' : `ACTIF (1:${1/NETHER_RAT})`;
    });

    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; maxGForce = 0; savePrecisionRecords(); });
    $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (dist/max/records) ?")) {
            localStorage.removeItem(P_RECORDS_KEY);
            distM = 0; timeMoving = 0; maxSpd = 0; maxGForce = 0;
            kSpd = 0; kUncert = 1000; kAltUncert = 1000; 
            location.reload(); 
        }
    });

    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        $('toggle-mode-btn').classList.toggle('active');
    });

    $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) {
            stopGPS();
            startGPS();
        }
    });
    
    // Tentative de démarrage GPS initial pour obtenir une position
    startGPS(); 
});
