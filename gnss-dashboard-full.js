// =================================================================
// FICHIER 1/2 : gnss-dashboard-core.js
// CŒUR DU SYSTÈME (Variables, EKF, Capteurs, Logique GPS/IMU)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- PARTIE 1 : CONSTANTES GLOBALES (Physiques, GPS, Temps, Astro) ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s)
const G_ACC_STD = 9.80665; // Gravité standard de la Terre (m/s²)
const M_EARTH = 5.972e24; // Masse de la Terre (kg)
const G_CONST = 6.67430e-11; // Constante gravitationnelle (N(m/kg)²)
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const NETHER_RATIO = 1 / 8; 

// Constantes de Conversion
const KMH_MS = 3.6; 
const KMS_MS = 1000; 
const AU_TO_M = 149597870700; 
const LIGHT_YEAR_TO_M = 9.461e15; 
// ... (Autres constantes de temps-lumière) ...

// Constantes Planétaires
const G_SURFACE_MOON = 1.625; const R_MOON = 1737400; const M_MOON = 7.34767e22; 
const G_SURFACE_MARS = 3.721; const R_MARS = 3389500; const M_MARS = 6.4171e23; 

// Constantes GPS et EKF
const MIN_DT = 0.05; 
const Q_NOISE = 0.001; 
const MIN_SPD = 0.001; 
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; 
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // À REMPLACER
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- VARIABLES GLOBALES (EKF & État) ---
let lat = 0, lon = 0, alt = 0, speed = 0, gpsTS = 0;
let kSpd = 0, kUncert = 1000, kAlt = 0, kAltUncert = 1000;
let lastTS = 0, lastFSpeed = 0, distM = 0;
let lastPos = null, lastAlt = 0; 
let sTime = null, timeMoving = 0, maxSpd = 0; 
let latestLinearAccelMagnitude = 0, maxGForce = 0; // Accel Longitudinale (Corrigée)
let latestRotationBeta = 0, latestRotationGamma = 0; // Pour le niveau à bulle
let wID = null, domID = null, weatherID = null;
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let currentCelestialBody = 'EARTH'; 
let G_ACC_LOCAL = G_ACC_STD; // Gravité locale dynamique

let P_RECORDS = { max_kUncert_min: 1000, max_acc_min: 1000, max_g_force_max: 0 };
const P_RECORDS_KEY = 'gnss_precision_records'; 

// ===========================================
// FONCTIONS UTILITAIRES ET GRAVITÉ DYNAMIQUE
// ===========================================

function calculateLocalGravity(altitude) {
    let R, M, g_surface;
    switch (currentCelestialBody) {
        case 'MOON': R = R_MOON; M = M_MOON; g_surface = G_SURFACE_MOON; break;
        case 'MARS': R = R_MARS; M = M_MARS; g_surface = G_SURFACE_MARS; break;
        case 'EARTH': default: R = R_E; M = M_EARTH; g_surface = G_ACC_STD; break;
    }
    
    if (altitude === null || isNaN(altitude) || currentCelestialBody !== 'EARTH') {
        G_ACC_LOCAL = g_surface; 
    } else {
        const h = altitude;
        G_ACC_LOCAL = G_CONST * M / Math.pow(R + h, 2);
    }
    
    if($('gravity-local')) $('gravity-local').textContent = `${G_ACC_LOCAL.toFixed(5)} m/s²`;
    return G_ACC_LOCAL;
}

function updateCelestialBody(body) {
    currentCelestialBody = body;
    calculateLocalGravity(alt); 
}

function loadPrecisionRecords() {
    // ... (Logique inchangée) ...
}

function savePrecisionRecords() {
    // ... (Logique inchangée) ...
}

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

function getEnvironmentFactor() {
    const env = $('environment-select').value;
    switch (env) {
        case 'FOREST': return 1.5;
        case 'METAL': return 2.5;
        case 'CONCRETE': return 3.0; 
        default: return 1.0; 
    }
}

function toReadableScientific(num) {
    if (num === 0) return "0.00";
    const exponent = Math.floor(Math.log10(Math.abs(num)));
    const mantissa = num / Math.pow(10, exponent);
    return `${mantissa.toFixed(2)}e+${exponent}`;
}

function getCDate() {
    return sTime ? new Date(new Date().getTime() - (sTime.getTime() - gpsTS)) : new Date();
}

// ===========================================
// CŒUR EKF, ZVU ET MISE À JOUR (updateDisp)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    if (typeof updateMap !== 'function') return;

    const acc = $('gps-accuracy-override').value !== '0.000000' ? parseFloat($('gps-accuracy-override').value) : pos.coords.accuracy;

    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    const currentAlt = pos.coords.altitude; 
    alt = currentAlt; 
    gpsTS = pos.timestamp;
    
    const nowTS = getCDate().getTime();
    const dt = lastTS === 0 ? MIN_DT : (nowTS - lastTS) / 1000;
    
    if (dt === 0) return; 

    let speedRaw;
    if (lastPos && dt > 0) {
        const d_horiz_raw = distanceCalc(lastPos.latitude, lastPos.longitude, lat, lon);
        speedRaw = d_horiz_raw / dt; 
    } else {
        speedRaw = pos.coords.speed !== null ? pos.coords.speed : 0; 
    }

    const currentHeading = pos.coords.heading !== null && !isNaN(pos.coords.heading) ? pos.coords.heading : 'N/A';
    lastTS = nowTS;

    const g_dynamic = calculateLocalGravity(currentAlt);
    
    // --- EKF (Filtre de Kalman) ---
    let kR = acc * acc * getEnvironmentFactor(); 
    let kGain = kUncert / (kUncert + kR);
    kSpd = kSpd + kGain * (speedRaw - kSpd); 
    kUncert = (1 - kGain) * kUncert;
    kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); 
    
    // --- ZVU (Mise à jour Zéro Vitesse) ---
    if (speedRaw < MIN_SPD * 10 && kSpd < MIN_SPD) { 
        kSpd = 0; 
    }
    
    // --- ACCÉLÉRATION & G-FORCE (Fusion Hybride IMU/EKF) ---
    let sSpdFE = Math.abs(kSpd);
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const accel_ekf = (dt > MIN_DT) ? (sSpdFE - lastFSpeed) / dt : 0;
    
    let accel_imu_signed = 0;
    const IMU_ACCEL_THRESHOLD = 0.1; 
    const FUSION_FACTOR = 0.7; 

    // Utilise la magnitude linéaire (corrigée de la gravité)
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
        speed3DInst = Math.sqrt(sSpdFE * sSpdFE + verticalSpeedRaw * verticalSpeedRaw);
    }
    
    // --- CALCUL DE LA DISTANCE (3D) ---
    if (lastPos && sSpdFE > MIN_SPD) { 
        const d_horiz_ekf = sSpdFE * dt;
        let d_vert = (verticalSpeedRaw) * dt;
        const d_3d = Math.sqrt(d_horiz_ekf * d_horiz_ekf + d_vert * d_vert); 
        distM += d_3d; 
        timeMoving += dt;
    }
    
    if (currentAlt !== null) lastAlt = currentAlt; 
    lastPos = { latitude: lat, longitude: lon };

    // --- MISE À JOUR DU DOM (Données principales) ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(4)} km/h`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(4)} km/h`;
    $('speed-3d-inst').textContent = `${(speed3DInst * KMH_MS).toFixed(4)} km/h`; 
    $('vertical-speed').textContent = `${verticalSpeedRaw.toFixed(2)} m/s`;
    $('accel-long').textContent = `${(accel_long).toFixed(3)} m/s ²`; 
    $('force-g-long').textContent = `${(accel_long / g_dynamic).toFixed(2)} G`;
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    // ... (Reste des mises à jour DOM) ...
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = currentAlt !== null ? `${currentAlt.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = acc !== null ? `${acc.toFixed(3)} m` : 'N/A';
    
    if (lat !== 0 && lon !== 0) updateMap(lat, lon); 
}

// ===========================================
// HANDLERS CAPTEURS IMU (CORRIGÉ)
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;

    // --- CORRECTION ACCÉLÉRATION VERTICALE ---
    // Utilise event.acceleration (linéaire, gravité soustraite par l'OS)
    const linearAccel = event.acceleration;
    
    if (linearAccel) {
        // Magnitude 3D (pour fusion EKF horizontale/longitudinale)
        latestLinearAccelMagnitude = Math.sqrt(linearAccel.x*linearAccel.x + linearAccel.y*linearAccel.y + linearAccel.z*linearAccel.z);
        
        // Accélération Z linéaire (Corrigée de la gravité)
        const linearZ = linearAccel.z;
        $('accel-vertical-imu').textContent = `${linearZ.toFixed(3)} m/s ²`;
        $('force-g-vertical').textContent = `${(linearZ / G_ACC_LOCAL).toFixed(2)} G`;

    } else {
        // Fallback si event.acceleration n'est pas supporté
        latestLinearAccelMagnitude = 0;
        $('accel-vertical-imu').textContent = 'N/A';
        $('force-g-vertical').textContent = 'N/A';
    }
}

// --- AJOUT : HANDLER NIVEAU À BULLE (ORIENTATION) ---
function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    latestRotationBeta = event.beta || 0;   // Tangage (Pitch)
    latestRotationGamma = event.gamma || 0; // Roulis (Roll)
    
    // Mise à jour du DOM
    $('pitch-angle').textContent = `${latestRotationBeta.toFixed(1)} °`;
    $('roll-angle').textContent = `${latestRotationGamma.toFixed(1)} °`;
}

// ===========================================
// LOGIQUE GPS ET DÉMARRAGE (CORRIGÉ)
// ===========================================

function continueGPSStart() {
    const opts = { enableHighAccuracy: currentGPSMode === 'HIGH_FREQ', timeout: 5000, maximumAge: 0 };
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = navigator.geolocation.watchPosition(updateDisp, (error) => { /* ... */ }, opts);
    if ($('freq-select')) $('freq-select').value = currentGPSMode; 
    sTime = sTime === null ? getCDate() : sTime; 
}

/** Gère la demande de permission de l'IMU (Accélération ET Orientation) */
function requestIMUPermissionAndStart() {
    let motionGranted = false;
    let orientationGranted = false;

    const startListeners = () => {
        if (motionGranted) window.addEventListener('devicemotion', handleDeviceMotion, true);
        if (orientationGranted) window.addEventListener('deviceorientation', handleDeviceOrientation, true);
        continueGPSStart();
    };

    // Demande 1: Accélération (Motion)
    const requestMotion = () => {
        if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission()
                .then(permissionState => {
                    motionGranted = (permissionState === 'granted');
                    if (!motionGranted) console.warn("Accès Accéléromètre refusé.");
                    requestOrientation(); // Passe à la demande suivante
                })
                .catch(err => {
                    console.error("Erreur permission DeviceMotion:", err);
                    requestOrientation(); // Tente quand même la suivante
                });
        } else {
            motionGranted = true; // Navigateurs non-iOS
            requestOrientation();
        }
    };

    // Demande 2: Orientation (Niveau à bulle)
    const requestOrientation = () => {
        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    orientationGranted = (permissionState === 'granted');
                    if (!orientationGranted) console.warn("Accès Orientation (Niveau à bulle) refusé.");
                    startListeners(); // Démarre avec ce qu'on a
                })
                .catch(err => {
                    console.error("Erreur permission DeviceOrientation:", err);
                    startListeners();
                });
        } else {
            orientationGranted = true; // Navigateurs non-iOS
            startListeners();
        }
    };

    // Lance la première demande
    requestMotion();
}

function startGPS() {
    if (wID === null) {
        requestIMUPermissionAndStart(); 
    }
    $('toggle-gps-btn').textContent = ' 7œ4„1‚5 ARRÊT GPS';
    $('toggle-gps-btn').style.backgroundColor = '#dc3545';
}

function stopGPS() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    
    // Arrête les écouteurs IMU
    window.removeEventListener('devicemotion', handleDeviceMotion, true);
    window.removeEventListener('deviceorientation', handleDeviceOrientation, true);

    $('toggle-gps-btn').textContent = ' 7œ4„1‚5 MARCHE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#28a745';
    }
// =================================================================
// FICHIER 2/2 : gnss-dashboard-ui-init.js
// MODULES UI & INITIALISATION (Astro, Carte, DOM)
// =================================================================

// Note: Ce fichier dépend des variables et fonctions globales définies dans gnss-dashboard-core.js

let map = null;
let marker = null;

/** Calcule et affiche l'heure solaire vraie (TST) et l'animation astro. */
function updateAstro(latitude, longitude) {
    if (latitude === 0 && longitude === 0 || typeof getCDate !== 'function' || typeof SunCalc === 'undefined') return;

    const now = getCDate();
    if (!now) return; // Si l'heure n'est pas encore synchronisée
    
    const times = SunCalc.getTimes(now, latitude, longitude);
    const pos = SunCalc.getPosition(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latitude, longitude);

    // 1. Calcul de l'Heure Solaire Vraie (TST)
    let fractionalDay = (now.getTime() - times.solarMidnight.getTime()) / (86400000);
    let tstHours = fractionalDay * 24;
    tstHours = tstHours % 24;

    const h = Math.floor(tstHours);
    const m = Math.floor((tstHours - h) * 60);
    const s = Math.floor(((tstHours - h) * 60 - m) * 60);

    // Affichage TST pour l'horloge Minecraft
    if($('time-minecraft')) $('time-minecraft').textContent = `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    
    // Affichage TST pour le champ de données
    if($('tst')) $('tst').textContent = `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;

    // 2. Animation Horloge (Style Minecraft TST)
    let clockRotation = (tstHours / 24) * 360; 
    
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');

    if(sunEl) sunEl.style.transform = `rotate(${clockRotation - 90}deg)`; 
    if(moonEl) moonEl.style.transform = `rotate(${(moonPos.azimuth + Math.PI) * R2D - 90}deg)`; // La lune suit sa propre rotation azimutale

    // 3. Logique de Couleur du Ciel (Saisonnière)
    const sunAltDeg = pos.altitude * R2D;
    let skyClass = '';
    const elevationThreshold = 0; 
    const isNight = sunAltDeg < elevationThreshold;

    if (!isNight) {
        skyClass = sunAltDeg > 20 ? 'sky-day' : 'sky-sunset'; 
    } else {
        skyClass = sunAltDeg > -12 ? 'sky-night-light' : 'sky-night'; 
    }
    
    // Applique la classe au body si le mode nuit n'est pas forcé
    if ($('toggle-mode-btn') && !$('toggle-mode-btn').classList.contains('active')) {
        document.body.className = skyClass;
    }

    // 3. Affichages Astro supplémentaires
    $('sun-elevation').textContent = `${sunAltDeg.toFixed(2)} °`;
    $('noon-solar').textContent = times.solarNoon.toLocaleTimeString();
    $('day-duration').textContent = `${((times.sunset.getTime() - times.sunrise.getTime()) / 3600000).toFixed(2)} h`;
    $('moon-phase-display').textContent = `${(moonIllumination.phase * 100).toFixed(1)}% | ${getMoonPhaseName(moonIllumination.phase)}`;
    $('ecliptic-long').textContent = `${(pos.eclipticLng * R2D).toFixed(2)} °`;
    
    // Calcul EOT (Équation du Temps)
    const meanSolarTime = (now.getTime() - times.solarMidnight.getTime()) / (86400000) * 24;
    const tstSec = h * 3600 + m * 60 + s;
    const lstSec = (meanSolarTime % 24) * 3600;
    const eotMin = (tstSec - lstSec) / 60;
    $('eot').textContent = `${eotMin.toFixed(2)} min`;
}

/** Nomme la phase lunaire. */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune";
    if (phase < 0.28) return "Premier Quartier";
    if (phase < 0.53) return "Pleine Lune";
    if (phase < 0.78) return "Dernier Quartier";
    return "Croissant/Gibbeuse";
}

// --- MÉTÉO ---

function getWeather() {
    if (lat === 0 || lon === 0 || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API Requis';
        $('pressure').textContent = 'API Requis';
        $('humidity').textContent = 'API Requis';
        // ... (autres champs météo)
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
                if(data.wind) $('wind-speed-ms').textContent = `${data.wind.speed.toFixed(1)} m/s`;
            }
        })
        .catch(err => console.error("Erreur de récupération météo :", err));
}

// --- CARTE LEAFLET ---

function initMap() {
    if (typeof L === 'undefined') {
        console.error("Leaflet n'est pas chargé.");
        return;
    }
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

// --- SYNCHRONISATION HORLOGE (NTP) ---

function syncH() {
    fetch(SERVER_TIME_ENDPOINT)
        .then(response => response.json())
        .then(data => {
            const apiTime = new Date(data.utc_datetime).getTime();
            const localTime = new Date().getTime();
            gpsTS = apiTime + (localTime - apiTime); 
            sTime = new Date();
        })
        .catch(err => {
            console.warn("Échec de la synchronisation NTP, utilise l'heure locale.", err);
            sTime = new Date();
        });
}


// --- INITIALISATION FINALE DU SYSTÈME ET LISTENERS ---

document.addEventListener('DOMContentLoaded', () => {
    // Note: loadPrecisionRecords() a été retiré car il n'était pas dans votre fichier de base
    initMap();
    syncH(); 
    
    if($('celestial-body-select')) updateCelestialBody($('celestial-body-select').value);
    
    // --- Intervalle de mise à jour lente DOM/Astro ---
    domID = setInterval(() => {
        const now = getCDate();
        if (now) {
            $('local-time').textContent = now.toLocaleTimeString();
            $('date-display').textContent = now.toLocaleDateString();
            if(sTime) $('time-elapsed').textContent = ((now.getTime() - sTime.getTime()) / 1000).toFixed(2) + ' s';
            $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
        }
        
        if (lat !== 0 && lon !== 0) {
            updateAstro(lat, lon); 
        }
    }, 1000); 
    
    // --- Intervalle pour la mise à jour Météo (30s) ---
    weatherID = setInterval(getWeather, 30000); 

    // --- Événements Utilisateur ---
    $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS() : stopGPS());
    
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '•0“5 Arrêt d\'urgence: ACTIF •0 4' : '•0“5 Arrêt d\'urgence: INACTIF •0 4';
        $('emergency-stop-btn').style.backgroundColor = emergencyStopActive ? '#f8d7da' : '#dc3545';
        if (emergencyStopActive) stopGPS();
    });
    
    if($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        const isActive = $('mode-nether').textContent.includes('ACTIF');
        $('mode-nether').textContent = isActive ? 'DÉSACTIVÉ (1:1)' : `ACTIF (1:${1/NETHER_RATIO})`;
        netherMode = !isActive; // Met à jour la variable d'état
    });

    if($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    if($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; maxGForce = 0; /* savePrecisionRecords(); */ });
    
    if($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (dist/max/records) ?")) {
            // localStorage.removeItem(P_RECORDS_KEY);
            distM = 0; timeMoving = 0; maxSpd = 0; maxGForce = 0;
            kSpd = 0; kUncert = 1000; kAltUncert = 1000; 
            location.reload(); 
        }
    });

    if($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        $('toggle-mode-btn').classList.toggle('active');
    });

    if($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) {
            stopGPS();
            startGPS();
        }
    });

    // Démarrage initial (attendra le clic utilisateur pour les permissions IMU)
    // startGPS(); // Remplacé par un clic utilisateur pour les permissions
});
