// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER PRINCIPAL (FULL)
// STRUCTURE FICHIERS : 
// - root: index.html, sw.js, leaflet.js, turf.min.js, math.min.js
// - lib/: ukf-lib.js, astro.js, ephem.js
// - api/: weather.js (Endpoint Vercel)
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

// Atmosphère Standard (ISA)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C
const RHO_SEA_LEVEL = 1.225;
const BARO_ALT_REF_HPA = 1013.25;

// WGS84 (Gravité locale)
const WGS84_A = 6378137.0;
const WGS84_G_EQUATOR = 9.7803253359;
const WGS84_BETA = 0.0053024;
const WGS84_E2 = 0.00669437999014;

// Configuration API & Système
const DOM_SLOW_UPDATE_MS = 2000; // Rafraîchissement lent (2s)
const PROXY_WEATHER_ENDPOINT = "/api/weather"; // Relatif à la racine (Vercel)
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- 2. ÉTAT GLOBAL ---

let ukf = null; // Instance ProfessionalUKF
let mapInstance = null; // Instance Leaflet
let mapMarker = null;   // Marqueur Leaflet
let mapCircle = null;   // Cercle de précision Leaflet

let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
let kAlt = 0.0;
let currentSpeed = 0.0;
let G_ACC = G_STD;

// Météo (Valeurs par défaut robustes)
let lastWeatherData = {
    tempC: 15.0,
    pressure_hPa: 1013.25,
    humidity_perc: 50,
    tempK: 288.15,
    air_density: 1.225,
    speed_of_sound: 340.29,
    timestamp: 0
};

let lServH = 0;
let lLocH = 0;


// --- 3. UTILITAIRES MATH & PHYSIQUE ---

const dataOrDefault = (val, dec, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) > 1e18) return 'N/A';
    return val.toFixed(dec) + suffix;
};

function getSpeedOfSound(tempK) {
    if (!tempK || tempK <= 0) return 340.29;
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

function getAirDensity(p_Pa, t_K, h_rel) {
    if (!t_K || !p_Pa) return 1.225;
    const T_c = t_K - 273.15;
    const Es = 610.7 * Math.pow(10, (7.5 * T_c) / (237.3 + T_c));
    const E = h_rel * Es; // Pression vapeur
    const R_VAP = 461.5;
    return (p_Pa - E) / (R_SPECIFIC_AIR * t_K) + (E / (R_VAP * t_K));
}

function updateGravityWGS84(lat, alt) {
    const phi = lat * D2R;
    const sinSq = Math.sin(phi) ** 2;
    const factor = (1 + WGS84_BETA * sinSq) / Math.sqrt(1 - WGS84_E2 * sinSq);
    // Correction d'altitude (Approximation Free Air)
    G_ACC = WGS84_G_EQUATOR * factor * (1 - (2 * alt / WGS84_A));
    if ($('gravity-local')) $('gravity-local').textContent = `${G_ACC.toFixed(4)} m/s²`;
}

// --- 4. GESTION SERVICE WORKER (PWA) ---

function registerServiceWorker() {
    if ('serviceWorker' in navigator) {
        // Le fichier sw.js est à la racine selon votre liste
        navigator.serviceWorker.register('./sw.js')
            .then(reg => console.log('SW: Enregistré 🟢', reg.scope))
            .catch(err => console.error('SW: Erreur 🔴', err));
    }
}


// --- 5. INITIALISATION CARTE LEAFLET ---

function initMap() {
    if (typeof L === 'undefined') {
        console.warn("Leaflet.js non chargé.");
        return;
    }
    
    // Initialisation sur Marseille par défaut ou dernière position
    mapInstance = L.map('map').setView([currentPosition.lat, currentPosition.lon], 16);

    // Tuiles OpenStreetMap
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors',
        maxZoom: 19
    }).addTo(mapInstance);

    // Icône personnalisée (optionnel, sinon défaut)
    mapMarker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(mapInstance);
    
    // Cercle de précision
    mapCircle = L.circle([currentPosition.lat, currentPosition.lon], {
        color: '#007bff',
        fillColor: '#007bff',
        fillOpacity: 0.2,
        radius: currentPosition.acc
    }).addTo(mapInstance);
}

function updateMap(lat, lon, acc) {
    if (!mapInstance || !mapMarker) return;
    
    const newLatLng = new L.LatLng(lat, lon);
    mapMarker.setLatLng(newLatLng);
    mapCircle.setLatLng(newLatLng);
    mapCircle.setRadius(acc);
    
    // Centrer la carte si l'utilisateur ne la déplace pas manuellement (optionnel)
    // mapInstance.panTo(newLatLng);
}


// --- 6. LOGIQUE API (Temps & Météo) ---

async function syncTime() {
    try {
        const res = await fetch(SERVER_TIME_ENDPOINT);
        const data = await res.json();
        lServH = new Date(data.utc_datetime).getTime();
        lLocH = Date.now();
    } catch (e) {
        lServH = 0; // Fallback local
    }
}

async function updateWeather() {
    // Si données récentes (< 5 min), on ne rappelle pas
    if (Date.now() - lastWeatherData.timestamp < 300000) return;

    try {
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Mise à jour... 🟡';
        // Appel au fichier api/weather.js via la route Vercel
        const res = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${currentPosition.lat}&lon=${currentPosition.lon}`);
        
        if (!res.ok) throw new Error("API Error");
        
        const data = await res.json();
        
        // Mise à jour des globales robustes
        lastWeatherData = {
            tempC: data.tempC,
            pressure_hPa: data.pressure_hPa,
            humidity_perc: data.humidity_perc,
            tempK: data.tempC + 273.15,
            timestamp: Date.now()
        };

        // Recalcul Physique
        const p_Pa = lastWeatherData.pressure_hPa * 100;
        lastWeatherData.air_density = getAirDensity(p_Pa, lastWeatherData.tempK, lastWeatherData.humidity_perc/100);
        lastWeatherData.speed_of_sound = getSpeedOfSound(lastWeatherData.tempK);

        updateWeatherDOM();
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Actif 🟢';

    } catch (e) {
        console.warn("Météo échouée, usage valeurs par défaut/précédentes.");
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Défaut (ISA) 🟡';
    }
}

function updateWeatherDOM() {
    const d = lastWeatherData;
    if ($('air-temp-c')) $('air-temp-c').textContent = d.tempC.toFixed(1) + ' °C';
    if ($('pressure-hpa')) $('pressure-hpa').textContent = d.pressure_hPa.toFixed(0) + ' hPa';
    if ($('air-density')) $('air-density').textContent = d.air_density.toFixed(3) + ' kg/m³';
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = d.speed_of_sound.toFixed(2) + ' m/s';
}


// --- 7. COEUR DU SYSTÈME : BOUCLES DE MISE À JOUR ---

// Callback GPS (Haute Fréquence)
function onPositionUpdate(pos) {
    const { latitude, longitude, accuracy, altitude, speed } = pos.coords;
    
    // Mise à jour objet courant
    currentPosition = {
        lat: latitude,
        lon: longitude,
        acc: accuracy,
        alt: altitude || 0,
        spd: speed || 0
    };

    // 1. Pipeline UKF (si chargé)
    if (ukf) {
        // Prédiction (Simulation IMU nulle si pas de capteurs réels)
        ukf.predict(0, 0, 0, 0, 0, 0);
        // Correction GNSS
        ukf.update(currentPosition, 'GNSS');
        
        const state = ukf.getState();
        
        // Mise à jour variables filtrées
        kAlt = state.alt;
        currentSpeed = state.speed;

        // DOM UKF
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(state.lat, 6, ' °');
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(state.lon, 6, ' °');
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(kAlt, 2, ' m');
        if ($('ukf-v-uncert')) $('ukf-v-uncert').textContent = dataOrDefault(state.kUncert, 3, ' m');
    } else {
        // Mode dégradé sans UKF
        kAlt = currentPosition.alt;
        currentSpeed = currentPosition.spd;
    }

    // 2. Physique temps réel
    updateGravityWGS84(currentPosition.lat, kAlt);

    // 3. Mise à jour DOM Rapide
    if ($('lat-val')) $('lat-val').textContent = dataOrDefault(currentPosition.lat, 6, ' °');
    if ($('lon-val')) $('lon-val').textContent = dataOrDefault(currentPosition.lon, 6, ' °');
    if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(currentSpeed * KMH_MS, 1, ' km/h');
    if ($('acc-horiz')) $('acc-horiz').textContent = dataOrDefault(currentPosition.acc, 1, ' m');

    // 4. Mise à jour Carte
    updateMap(currentPosition.lat, currentPosition.lon, currentPosition.acc);
}

// Boucle Lente (2s) : Astro, Horloge, Météo
function slowLoop() {
    const now = lServH > 0 ? new Date(Date.now() + (lServH - lLocH)) : new Date();

    // 1. Horloge
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString();
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();

    // 2. Astro (lib/astro.js)
    if (typeof getAstroData === 'function') {
        const astro = getAstroData(now, currentPosition.lat, currentPosition.lon);
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astro.sun.altitude * R2D, 1, ' °');
        if ($('moon-phase-name')) $('moon-phase-name').textContent = astro.moon.phaseName;
    }

    // 3. Appel Météo (vérifie le délai interne)
    updateWeather();
}


// --- 8. INITIALISATION (Load Event) ---

window.addEventListener('load', () => {
    console.log("Système : Démarrage...");

    // 1. Enregistrer SW
    registerServiceWorker();

    // 2. Initialiser UKF (lib/ukf-lib.js)
    if (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') {
        try {
            ukf = new ProfessionalUKF();
            if ($('ekf-status')) $('ekf-status').textContent = 'Initialisé 🟢';
        } catch (e) {
            console.error(e);
            if ($('ekf-status')) $('ekf-status').textContent = 'Erreur Lib 🔴';
        }
    }

    // 3. Initialiser Carte (leaflet.js)
    initMap();

    // 4. Initialiser GPS
    if (navigator.geolocation) {
        if ($('gnss-status')) $('gnss-status').textContent = 'Recherche... 🟡';
        navigator.geolocation.watchPosition(
            onPositionUpdate,
            (err) => { if ($('gnss-status')) $('gnss-status').textContent = `Erreur ${err.code} 🔴`; },
            { enableHighAccuracy: true, maximumAge: 0 }
        );
    } else {
        alert("Géolocalisation non supportée.");
    }

    // 5. Initialiser Temps & Météo par défaut
    syncTime();
    updateWeatherDOM(); // Affiche les valeurs par défaut au démarrage

    // 6. Lancer boucle lente
    setInterval(slowLoop, DOM_SLOW_UPDATE_MS);
    
    // Listeners UI
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70;
        if ($('mass-display')) $('mass-display').textContent = currentMass.toFixed(1) + ' kg';
    });
});
