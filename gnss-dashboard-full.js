// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER PRINCIPAL (FULL)
// VERSION FINALE ET ROBUSTE : Gestion des échecs GPS/NTP/IMU/API
// =================================================================

// --- 1. CONFIGURATION ET CONSTANTES ---

const $ = id => document.getElementById(id);

// Constantes Physiques & WGS84
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;
const C_L = 299792458;
const G_STD = 9.80665;
const R_SPECIFIC_AIR = 287.058;
const GAMMA_AIR = 1.4;
const WGS84_A = 6378137.0;
const WGS84_G_EQUATOR = 9.7803253359;
const WGS84_BETA = 0.0053024;
const WGS84_E2 = 0.00669437999014;

// Atmosphère Standard (ISA Fallback)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C
const RHO_SEA_LEVEL = 1.225;
const BARO_ALT_REF_HPA = 1013.25;

// Configuration API & Système
const DOM_SLOW_UPDATE_MS = 2000;
const PROXY_WEATHER_ENDPOINT = "/api/weather"; // Endpoint Vercel
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- 2. ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---

let ukf = null;
let mapInstance = null;
let mapMarker = null;
let mapCircle = null;

let isGpsPaused = false; // État du bouton MARCHE/ARRÊT GPS
let gpsWatchId = null;

// Position de travail par défaut (Marseille) pour débloquer Astro/Météo
let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 100.0, spd: 0.0, alt: 0.0 };
let kAlt = 0.0;
let currentSpeed = 0.0;
let G_ACC = G_STD;

let lastWeatherData = {
    tempC: 15.0,
    pressure_hPa: 1013.25,
    humidity_perc: 50, // Ajout de l'humidité par défaut
    tempK: 288.15,
    air_density: 1.225,
    speed_of_sound: 340.29,
    timestamp: 0
};

let lServH = 0; // Temps serveur (NTP)
let lLocH = 0;  // Temps local au moment de la synchro

// --- 3. UTILITAIRES MATH, AFFICHAGE ET PHYSIQUE ---

const dataOrDefault = (val, dec, suffix = '') => {
    // Affiche 'N/A' si valeur non valide, sinon valeur formatée
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) > 1e18) {
        // Pour les décimales, on peut afficher un tiret si N/A pour la lisibilité
        return (dec === 0 ? 'N/A' : '--.--') + suffix;
    }
    return val.toFixed(dec) + suffix;
};

// Modèle Gravité WGS84
function updateGravityWGS84(lat, alt) {
    const phi = lat * D2R;
    const sinSq = Math.sin(phi) ** 2;
    const factor = (1 + WGS84_BETA * sinSq) / Math.sqrt(1 - WGS84_E2 * sinSq);
    G_ACC = WGS84_G_EQUATOR * factor * (1 - (2 * alt / WGS84_A));
    
    // CORRECTION CRITIQUE : Affichage de la gravité locale
    if ($('gravity-local')) $('gravity-local').textContent = `${G_ACC.toFixed(4)} m/s²`;
}

// Modèle Physique
function getSpeedOfSound(tempK) {
    if (!tempK || tempK <= 0) return 340.29; // Fallback ISA
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

// --- 4. GESTION TEMPS (NTP) ---

function getCDate(serverTimeMs, localTimeMs) {
    if (serverTimeMs > 0 && localTimeMs > 0) {
        const offset = serverTimeMs - localTimeMs;
        return new Date(Date.now() + offset);
    }
    return new Date(); // Fallback sur l'heure système locale
}

async function syncTime() {
    try {
        const res = await fetch(SERVER_TIME_ENDPOINT, { signal: AbortSignal.timeout(3000) }); // Ajout d'un timeout
        if (!res.ok) throw new Error("API NTP HS");
        const data = await res.json();
        lServH = new Date(data.utc_datetime).getTime();
        lLocH = Date.now();
        if ($('local-time')) $('local-time').textContent = getCDate(lServH, lLocH).toLocaleTimeString('fr-FR');
        if ($('date-display')) $('date-display').textContent = getCDate(lServH, lLocH).toLocaleDateString('fr-FR');
        console.log("NTP: Synchro OK 🟢");
    } catch (e) {
        lServH = 0;
        lLocH = 0;
        // CORRECTION : Affichage de l'heure système par défaut
        if ($('local-time')) $('local-time').textContent = new Date().toLocaleTimeString('fr-FR') + ' (LOCAL)';
        if ($('date-display')) $('date-display').textContent = new Date().toLocaleDateString('fr-FR');
        console.warn("NTP: Échec, utilisation heure système 🟡");
    }
}

// --- 5. GESTION MÉTÉO (API) ---

async function updateWeather() {
    // Si données récentes (< 5 min), on ne rappelle pas
    if (Date.now() - lastWeatherData.timestamp < 300000) return;

    try {
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Mise à jour... 🟡';
        const res = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${currentPosition.lat}&lon=${currentPosition.lon}`);

        if (!res.ok) throw new Error("API Error");

        const data = await res.json();

        // Mise à jour des globales
        lastWeatherData = {
            tempC: data.tempC,
            pressure_hPa: data.pressure_hPa,
            humidity_perc: data.humidity_perc,
            tempK: data.tempC + 273.15,
            timestamp: Date.now()
        };

        const p_Pa = lastWeatherData.pressure_hPa * 100;
        lastWeatherData.air_density = getAirDensity(p_Pa, lastWeatherData.tempK, lastWeatherData.humidity_perc/100);
        lastWeatherData.speed_of_sound = getSpeedOfSound(lastWeatherData.tempK);

        updateWeatherDOM();
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Actif 🟢';

    } catch (e) {
        // En cas d'échec, on ré-injecte les valeurs ISA par défaut pour éviter le NaN
        updateWeatherDOM(lastWeatherData, true);
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Défaut (ISA) 🟡';
    }
}

function updateWeatherDOM() {
    const d = lastWeatherData;
    if ($('air-temp-c')) $('air-temp-c').textContent = d.tempC.toFixed(1) + ' °C';
    if ($('pressure-hpa')) $('pressure-hpa').textContent = d.pressure_hPa.toFixed(0) + ' hPa';
    // CORRECTION : Affichage de l'humidité
    if ($('humidity-perc')) $('humidity-perc').textContent = d.humidity_perc + ' %';
    if ($('air-density')) $('air-density').textContent = d.air_density.toFixed(3) + ' kg/m³';
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = d.speed_of_sound.toFixed(2) + ' m/s';
}

// --- 6. GESTION CAPTEURS (GPS & IMU) ---

// 6.1. IMU (Accéléromètre/Gyroscope)
function handleMotion(event) {
    const acc = event.accelerationIncludingGravity;
    const gyro = event.rotationRate;

    // Mise à jour des accélérations (pour l'affichage)
    if (acc) {
        if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(acc.x, 2);
        if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(acc.y, 2);
        if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(acc.z, 2);
    }
    
    // Envoi des données à l'UKF
    if (ukf && acc) {
        const ax = acc.x || 0;
        const ay = acc.y || 0;
        const az = acc.z || 0;
        const gx = gyro ? (gyro.alpha || 0) : 0;
        const gy = gyro ? (gyro.beta || 0) : 0;
        const gz = gyro ? (gyro.gamma || 0) : 0;
        ukf.predict(ax, ay, az, gx, gy, gz);
    }
}

function activateDeviceMotion() {
    // Gestion des permissions iOS 13+ (nécessite HTTPS)
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(response => {
                if (response === 'granted') {
                    window.addEventListener('devicemotion', handleMotion);
                    if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢';
                    alert("Capteurs de mouvement activés.");
                } else {
                    alert("Permission IMU refusée.");
                }
            })
            .catch(err => {
                console.error("Erreur d'activation IMU:", err);
                alert("Erreur: Impossible d'activer les capteurs IMU.");
            });
    } else {
        // Pour Android / autres
        window.addEventListener('devicemotion', handleMotion);
        if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢';
    }
}

// 6.2. GPS
function onPositionUpdate(pos) {
    if (isGpsPaused) return; // Ne pas traiter si en pause

    const { latitude, longitude, accuracy, altitude, speed } = pos.coords;

    currentPosition = {
        lat: latitude,
        lon: longitude,
        acc: accuracy,
        alt: altitude || 0,
        spd: speed || 0
    };

    // 1. Pipeline UKF
    if (ukf) {
        // Correction GNSS
        ukf.update(currentPosition, 'GNSS');

        const state = ukf.getState();
        kAlt = state.alt;
        currentSpeed = state.speed;

        // Affichage Position EKF
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(state.lat, 6, ' °');
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(state.lon, 6, ' °');
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(kAlt, 2, ' m');
        if ($('ekf-status')) $('ekf-status').textContent = 'Fusion 🟢';

    } else {
        kAlt = currentPosition.alt;
        currentSpeed = currentPosition.spd;
    }

    // 2. Physique temps réel & Affichage Vitesse
    updateGravityWGS84(currentPosition.lat, kAlt);
    if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(currentSpeed * KMH_MS, 1, ' km/h');
    if ($('acc-horiz')) $('acc-horiz').textContent = dataOrDefault(currentPosition.acc, 1, ' m');
    if ($('gnss-status')) $('gnss-status').textContent = 'Acquisition 🟢';

    // 3. Mise à jour Carte (Leaflet)
    if (typeof updateMap === 'function') updateMap(currentPosition.lat, currentPosition.lon, currentPosition.acc);
}

function initGPS() {
    if (navigator.geolocation) {
        if (gpsWatchId) navigator.geolocation.clearWatch(gpsWatchId);
        if ($('gnss-status')) $('gnss-status').textContent = 'Recherche... 🟡';

        gpsWatchId = navigator.geolocation.watchPosition(
            onPositionUpdate,
            (err) => { if ($('gnss-status')) $('gnss-status').textContent = `Erreur GPS (${err.code}) 🔴`; },
            { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
        );
    } else {
        if ($('gnss-status')) $('gnss-status').textContent = 'Non supporté ❌';
    }
}

function toggleGpsPause() {
    isGpsPaused = !isGpsPaused;
    const btn = $('gps-toggle-btn');
    if (btn) btn.textContent = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
    if (isGpsPaused) {
        if ($('gnss-status')) $('gnss-status').textContent = 'PAUSE ⏸️';
    } else {
        initGPS(); // Redémarre l'acquisition
    }
}


// --- 7. BOUCLE LENTE (ASTRO & MÉTÉO) ---

function slowLoop() {
    const now = getCDate(lServH, lLocH);

    // 1. Horloge & Date
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');

    // 2. Astro (lib/astro.js) - Nécessite la date et la position
    if (typeof getAstroData === 'function') {
        const astro = getAstroData(now, currentPosition.lat, currentPosition.lon);
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astro.sun.altitude * R2D, 1, ' °');
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astro.sun.azimuth * R2D, 1, ' °');
        if ($('moon-phase-name')) $('moon-phase-name').textContent = astro.moon.phaseName;
        if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astro.moon.fraction * 100, 1, ' %');
    }

    // 3. Appel Météo (vérifie le délai interne)
    updateWeather();
}

// --- 8. INITIALISATION (Load Event) ---

window.addEventListener('load', () => {

    // 1. Initialiser UKF (math.min.js et lib/ukf-lib.js doivent être chargés avant)
    if (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') {
        try {
            ukf = new ProfessionalUKF();
            if ($('ekf-status')) $('ekf-status').textContent = 'Initialisé 🟢';
        } catch (e) {
            if ($('ekf-status')) $('ekf-status').textContent = 'Erreur Lib 🔴';
            console.error("UKF non initialisé:", e);
        }
    } else {
        if ($('ekf-status')) $('ekf-status').textContent = 'DÉSACTIVÉ 🔴';
    }
    
    // 2. Initialiser la Carte (Leaflet)
    if (typeof L !== 'undefined') {
        initMap();
    } else {
        console.warn("Leaflet.js non chargé.");
    }

    // 3. Démarrer la synchro NTP (pour avoir l'heure avant tout calcul Astro)
    syncTime();

    // 4. Initialiser GPS et Météo (l'ordre est important)
    initGPS();
    updateWeatherDOM(); // Affiche les valeurs ISA par défaut au démarrage
    
    // 5. Initialisation des constantes et Gravité de base
    if ($('const-c')) $('const-c').textContent = `${C_L} m/s`;
    if ($('gravity-base')) $('gravity-base').textContent = `${G_STD.toFixed(4)} m/s²`;
    updateGravityWGS84(currentPosition.lat, kAlt);

    // 6. Lancer boucle lente
    setInterval(slowLoop, DOM_SLOW_UPDATE_MS);

    // 7. Listeners d'événements
    if ($('activate-sensors-btn')) $('activate-sensors-btn').addEventListener('click', activateDeviceMotion);
    if ($('gps-toggle-btn')) $('gps-toggle-btn').addEventListener('click', toggleGpsPause);

    // ... (Ajouter ici vos autres listeners comme Réinit. Dist., etc. si non inclus)
});
