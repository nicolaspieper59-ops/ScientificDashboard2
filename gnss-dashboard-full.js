// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes, Globales & Utilitaires)
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

// Seuil plancher théorique pour l'incertitude d'état EKF (P)
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; // 5e-5 m² (implémentation du Plancher d'Incertitude EKF)
const MIN_SPD = 0.001; // Seuil de vitesse minimale (1 mm/s)

// --- VARIABLES GLOBALES (État du Système) ---
let lat = 0, lon = 0, alt = 0, speed = 0, hdop = 1000, gpsTS = 0;
let wID = null, domID = null, weatherID = null, emergencyStopActive = false;
let sTime = null, lastTS = 0, lastPos = null, timeMoving = 0;
let distM = 0;
let currentGPSMode = 'HIGH_FREQ'; 
let netherMode = false;
let G_ACC_LOCAL = G_ACC; // Gravité locale (utilisée par handleDeviceMotion)
let mass = 70.0; // Masse par défaut (kg)

// --- VARIABLES EKF (Filtre de Kalman Étendu) ---
let kSpd = 0;       // Vitesse stable EKF (m/s)
let kUncert = 1000; // Incertitude horizontale EKF (m²) - commence haut
let kAlt = 0;       // Altitude stable EKF (m)
let kAltUncert = 1000; // Incertitude verticale EKF (m²) - commence haut
let lastFSpeed = 0; // Dernière vitesse filtrée pour l'accélération
let maxSpd = 0;     // Vitesse max (m/s)
let maxGForce = 0;  // G-Force Max (G) - Initialisée à 0

// --- VARIABLES DE CAPTEURS IMU ---
let latestLinearAccelMagnitude = 0;
let latestAccelX = 0, latestAccelY = 0, latestAccelZ = 0;

// --- VARIABLES RECORDS DE PRÉCISION ---
let P_RECORDS = {
    max_kUncert_min: 1000, // Incertitude horizontale minimale atteinte (m²)
    max_acc_min: 1000,     // Précision GPS brute minimale atteinte (m)
    max_g_force_max: 0     // Force G maximale absolue enregistrée (G)
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
            
            // Appliquer le record de G-Force max au compteur courant
            maxGForce = P_RECORDS.max_g_force_max;
        }
    } catch (e) {
        console.error("Erreur lors du chargement des records de précision:", e);
    }
}

function savePrecisionRecords() {
    try {
        // Mettre à jour la G-Force max actuelle avant de sauvegarder
        // Note: totalGForce (IMU) doit mettre à jour maxGForce dans part2.js
        P_RECORDS.max_g_force_max = maxGForce; 
        localStorage.setItem(P_RECORDS_KEY, JSON.stringify(P_RECORDS));
    } catch (e) {
        console.error("Erreur lors de la sauvegarde des records de précision:", e);
    }
}
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js (Logique EKF, Capteurs & API Météo)
// =================================================================

// Les fonctions EKF, setGPSMode, updateDisp, handleDeviceMotion et getWeather sont ici.

/** Simulation de la correction GPS théorique parfaite (Correction Théorique Automatique au Démarrage). */
function simulateBestCorrection() {
    // Correction uniquement si les coordonnées sont connues après le démarrage
    if (lat === 0 && lon === 0) {
        // Cette initialisation est l'EKF Correction Théorique au Démarrage
        kUncert = MIN_UNCERT_FLOOR; 
        kAltUncert = MIN_UNCERT_FLOOR; 
        return; 
    }

    const IDEAL_ACCURACY = 0.00001; // 0.01 mm

    const mockBestCorrectionPos = {
        coords: {
            latitude: lat,
            longitude: lon,
            altitude: kAlt, 
            accuracy: IDEAL_ACCURACY, 
            speed: kSpd, 
            altitudeAccuracy: IDEAL_ACCURACY
        },
        timestamp: new Date().getTime()
    };

    kUncert = MIN_UNCERT_FLOOR; 
    kAltUncert = MIN_UNCERT_FLOOR; 

    updateDisp(mockBestCorrectionPos); 
    
    // (Retiré l'alerte pour ne pas bloquer l'interface au démarrage)
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
        // Logique d'autorisation des capteurs IMU (critique pour iOS/Safari)
        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        continueGPSStart();
                    } else {
                        console.warn("Accès aux capteurs de mouvement refusé. Certaines données seront indisponibles.");
                        continueGPSStart(); // On lance quand même le GPS
                    }
                })
                .catch(err => {
                    console.error("Erreur d'autorisation DeviceMotion:", err);
                    continueGPSStart(); 
                });
        } else {
            // Pour Android, Desktop, ou si la fonction d'autorisation n'existe pas
            continueGPSStart();
        }
    }
}


/** Handler des données GPS et du filtre EKF (Correction Automatique de Dérive GPS). */
function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    const acc = $('gps-accuracy-override').value !== '0.000000' ? parseFloat($('gps-accuracy-override').value) : pos.coords.accuracy;

    // --- LOGIQUE DE CORRECTION AUTOMATIQUE À LA FIN DE LA DÉRIVE ---
    const hasGPSFix = acc !== null && acc < 50; // Considère un fix si précision < 50m
    const wasDrifting = kUncert > 1.0; // Considère une dérive si l'incertitude est > 1m²

    if (wasDrifting && hasGPSFix) {
        // Correction Automatique de Dérive GPS
        console.log("Correction automatique déclenchée : Signal GPS revenu après dérive.");
        
        // Caler la position EKF sur la nouvelle mesure GPS
        lat = pos.coords.latitude; 
        lon = pos.coords.longitude;
        kAlt = pos.coords.altitude !== null ? pos.coords.altitude : kAlt; 
        
        // Annuler l'incertitude accumulée (réinitialiser la dérive d'état)
        kUncert = MIN_UNCERT_FLOOR; // Utilisation du Plancher d'Incertitude EKF
        kAltUncert = MIN_UNCERT_FLOOR; 
    }
    // ---------------------------------------------------

    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    alt = pos.coords.altitude;
    speed = pos.coords.speed !== null ? pos.coords.speed : 0;
    gpsTS = pos.timestamp;
    
    const nowTS = getCDate().getTime();
    const dt = lastTS === 0 ? MIN_DT : (nowTS - lastTS) / 1000;
    lastTS = nowTS;

    // --- EKF (Simplified 1D Speed/Position Filter) ---
    // (Le reste de l'EKF complet serait ici, utilisant Q_NOISE et kUncert)
    
    // Simplification EKF pour la vitesse (EKF Speed)
    let kR = acc * acc * getEnvironmentFactor(); // Bruit de mesure
    let kGain = kUncert / (kUncert + kR);
    kSpd = kSpd + kGain * (speed - kSpd);
    kUncert = (1 - kGain) * kUncert;
    kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); // Plancher d'Incertitude EKF

    // --- ACCÉLÉRATION & G-FORCE MAX ---
    let sSpdFE = Math.abs(kSpd);
    let accel_long = (dt > MIN_DT) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;

    // G-Force Max (longitudinale)
    const currentGForceLong = Math.abs(accel_long / G_ACC); 
    if (currentGForceLong > maxGForce) maxGForce = currentGForceLong; // Mise à jour du record de G-Force Max

    // --- MISE À JOUR DOM & nm/s ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(4)} km/h`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;

    // Affichage de la Vitesse avec la précision nm/s
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`; 
    
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G | Max: ${maxGForce.toFixed(2)} G`;

    // --- MISE À JOUR DES RECORDS DE PRÉCISION ---
    if (acc !== null && acc < P_RECORDS.max_acc_min) {
        P_RECORDS.max_acc_min = acc;
    }
    if (kUncert < P_RECORDS.max_kUncert_min) {
        P_RECORDS.max_kUncert_min = kUncert;
    }
    
    // Logique de distance et de mouvement
    if (lastPos) {
        const d = distanceCalc(lastPos.latitude, lastPos.longitude, lat, lon);
        distM += d;
        if (sSpdFE > MIN_SPD) {
            timeMoving += dt;
        }
    }
    lastPos = { latitude: lat, longitude: lon };
    
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
}


/** Handler pour les capteurs de mouvement (IMU) */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const accel = event.accelerationIncludingGravity;

    latestAccelX = accel.x || 0;
    latestAccelY = accel.y || 0;
    latestAccelZ = accel.z || 0;
    
    // Accélération linéaire sans la gravité
    const linearAccel = event.acceleration;
    if (linearAccel) {
        latestLinearAccelMagnitude = Math.sqrt(linearAccel.x*linearAccel.x + linearAccel.y*linearAccel.y + linearAccel.z*linearAccel.z);
    } else {
        latestLinearAccelMagnitude = 0;
    }
    
    // Mise à jour de la G-Force Max totale (IMU)
    // Note: Utilise latestLinearAccelMagnitude pour la G-Force liée au mouvement (si disponible)
    const totalGForce = latestLinearAccelMagnitude / G_ACC_LOCAL; 

    if (totalGForce > maxGForce) {
        maxGForce = totalGForce; // MET À JOUR LE RECORD COURANT
    }
    
    // Mise à jour de l'affichage vertical
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
                // ... (Logique de mise à jour des éléments DOM Météo)
                $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
                $('pressure').textContent = `${data.main.pressure} hPa`;
                $('humidity').textContent = `${data.main.humidity} %`;
                // ... (autres mises à jour)
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

// Les fonctions updateAstro, updateMinecraftClock, syncH et tous les addEventListener sont ici.

function updateAstro(latitude, longitude) {
    if (typeof SunCalc === 'undefined') return;
    const now = getCDate();
    const sunTimes = SunCalc.getTimes(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const moonPhaseName = getMoonPhaseName(moonIllumination.phase);

    // ... (Logique de mise à jour des éléments DOM Astro)
    $('sun-elevation').textContent = `${(sunTimes.solarAngle * R2D).toFixed(2)} °`;
    $('noon-solar').textContent = sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';
    $('moon-phase-display').textContent = moonPhaseName; // AJOUT DE LA PHASE DE LA LUNE
}

function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase >= 0.97) return 'Nouvelle Lune';
    if (phase < 0.22) return 'Premier Croissant';
    if (phase < 0.28) return 'Premier Quartier';
    if (phase < 0.47) return 'Lune Gibbeuse Croissante';
    if (phase < 0.53) return 'Pleine Lune';
    if (phase < 0.72) return 'Lune Gibbeuse Décroissante';
    if (phase < 0.78) return 'Dernier Quartier';
    return 'Dernier Croissant';
}

function updateMinecraftClock(tstTimeMs) {
    // ... (Logique de rotation de l'horloge)
    // Placeholder for visual updates
    $('time-minecraft').textContent = new Date(tstTimeMs).toTimeString().slice(0, 8);
}

function syncH() {
    // ... (Logique de synchronisation de l'heure NTP)
}

document.addEventListener('DOMContentLoaded', () => {
    // --- Initialisation du Système EKF & Persistence ---
    loadPrecisionRecords();
    simulateBestCorrection(); // Correction Théorique Automatique au Démarrage

    // --- Événements GPS/Capteurs IMU ---
    // Initialisation du listener IMU (nécessite l'autorisation dans startGPS sur mobile)
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté ou activé sur cet appareil/navigateur.");
    } 

    // --- Initialisation des intervalles ---
    syncH(); // Démarrage de la synchronisation de l'heure
    startGPS(); // Démarrage initial du GPS (inclut l'autorisation IMU)
    getWeather(); // Premier appel Météo

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

    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
    });

    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? "•0“5 Arrêt d'urgence: ACTIF •0 4" : "•0“5 Arrêt d'urgence: INACTIF •0 4";
    });

    // Réinitialisation de tous les records (avec G-Force Max)
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser? (Distance, Max, Kalman)")) { 
            distM = 0; maxSpd = 0; maxGForce = 0; // Réinitialisation de la G-Force Max
            kSpd = 0; kUncert = MIN_UNCERT_FLOOR; kAlt = 0; kAltUncert = MIN_UNCERT_FLOOR; timeMoving = 0; lastFSpeed = 0;
            savePrecisionRecords(); // Sauvegarder les records à zéro (si le record max n'est pas en mémoire)
            // if (tracePolyline) tracePolyline.setLatLngs([]); // Assurez-vous que cette fonction est dans la partie Leaflet/Map
        } 
    });

    // Réinitialisation de la vitesse max (Vitesse Max Session)
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => {
        if (emergencyStopActive) return; 
        maxSpd = 0;
        maxGForce = 0;
        P_RECORDS.max_g_force_max = 0;
        savePrecisionRecords(); 
        alert("Vitesse Max et G-Force Max réinitialisées.");
    });
    
    // Événement pour sauvegarder les records avant de quitter la page
    window.addEventListener('beforeunload', savePrecisionRecords);
});
