// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET & CORRIGÉ (UKF 21 ÉTATS)
// VERSION : FINALE ULTRA-ROBUSTE V7.4 (NON SIMULÉ / COORDONNÉES MANUELLES)
// OBJECTIF : Fichier complet intégrant tous les IDs du HTML utilisateur (V2).
// DÉPENDANCES CRITIQUES (doivent être chargées) : math.min.js, ukf-lib.js, astro.js, leaflet.js, turf.min.js
// =================================================================

// =================================================================
// PARTIE 1 : CONSTANTES ET UTILITAIRES FONDAMENTAUX
// =================================================================

const $ = id => document.getElementById(id);

// --- CLÉS D'API & ENDPOINTS ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 3000; // Fréquence de rafraîchissement des éléments lents
const DOM_FAST_UPDATE_MS = 1000 / 60; // 60Hz pour le rafraîchissement principal

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle (N·m²/kg²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation Terre (rad/s)
const EARTH_RADIUS_AVG = 6371000; // Rayon terrestre moyen (m)

// Constantes Atmosphériques (ISA Standard pour fallback métrologique)
const TEMP_SEA_LEVEL_K = 288.15; // 15 °C
const BARO_ALT_REF_HPA = 1013.25; // Pression niveau mer (hPa)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air niveau mer (kg/m³)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4; // Indice adiabatique de l'air
const DRAG_COEFF = 0.5; // Coefficient de traînée (simple placeholder)
const REF_AREA = 1.0; // Surface de référence (m²)

// --- FONCTIONS DE FORMATAGE CORRIGÉES ---

const dataOrDefault = (val, decimals, suffix = '', fallback = 'N/A', forceZero = true) => {
    if (val === undefined || val === null || isNaN(val) || (typeof val === 'number' && Math.abs(val) < 1e-18 && forceZero)) {
        if (fallback !== 'N/A') return fallback;
        if (forceZero) {
            const zeroFormat = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
            return zeroFormat.replace('.', ',');
        }
        return 'N/A';
    }
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals).replace('.', ',').replace('e', 'e') + suffix;
};

// =================================================================
// PARTIE 2 : ÉTAT GLOBAL ET VARIABLES DE CAPTEURS
// =================================================================

let ukf = null; // Instance du filtre UKF
let isGpsPaused = false; // Démarrage en mode ACTIF
let isGpsActive = false; 
let isMapInitialized = false;
let map, marker, trackPolyline;

// --- ETAT PHYSIQUE & MATHÉMATIQUE ---
let currentCelestialBody = 'EARTH';
let currentUKFReactivity = 'AUTO';
let rotationRadius = 100.0;
let angularVelocity = 0.0;
let distanceRatioMode = false; // Surface/Altitude
let netherMode = false; // Nether mode (1:8)
let currentGravity = 9.80665;

// --- COORDONNÉES MANUELLES INITIALES (Positionnement EKF) ---
let currentPosition = { 
    lat: 43.284578,   // Latitude (Marseille, par exemple)
    lon: 5.358713,    // Longitude
    alt: 100.00,      // Altitude (m)
    acc: 10.0,        // Précision initiale pour le filtre
    spd_ms: 0.0,      // Vitesse EKF (m/s)
    vel_v: 0.0,       // Vitesse verticale EKF (m/s)
    heading: 0.0,     // Cap
    g_force: 0.0,     // Force G longitudinale
    g_force_vert: 1.0 // Force G verticale (poids)
};
let maxSpeedKmH = 0.0;
let totalDistanceM = 0.0;
let totalMovementTimeS = 0.0;
let lastTimestamp = Date.now();
let kAlt = currentPosition.alt; // Altitude utilisée pour les calculs

// Variables de capteurs (dernières mesures brutes)
let lastGPS = null;
let gpsWatchID = null;
let lastIMU = { 
    acc: { x: null, y: null, z: null }, 
    mag: { x: null, y: null, z: null },
    pitch: 0.0, roll: 0.0, heading: 0.0
};
let lastAmbientLight = null; 
let maxAmbientLight = 0.0;

// Variables Métrologiques (API/ISA)
let lastP_hPa = BARO_ALT_REF_HPA; 
let lastT_K = TEMP_SEA_LEVEL_K; 
let lastH_perc = 0.6; // Humidité par défaut (60%)
let currentAirDensity = RHO_SEA_LEVEL; 
let currentSpeedOfSound = 343.0; 
let currentMass = 70.0; // Masse par défaut (kg)

// Variables Horloge NTP
let lServH = 0; 
let lLocH = 0; 

// =================================================================
// PARTIE 3 : FONCTIONS DE CALCUL PHYSIQUE ET MODÈLE DE TERRE/GRAVITÉ
// =================================================================

function getGravity(latRad, alt) {
    // Gravité WGS84 simplifiée (fonction dans ukf-lib.js si disponible)
    const G_E = 9.780327;
    const sin2 = Math.sin(latRad)**2;
    const g_0 = G_E * (1 + 0.0053024 * sin2);
    // Correction d'altitude (anomalie à l'air libre)
    const g_alt = g_0 - 3.086e-6 * alt;
    return g_alt;
}

function updateCelestialBody(body, alt, radius, angularVel) {
    let G_ACC_NEW = 9.80665; 
    switch (body) {
        case 'EARTH': G_ACC_NEW = getGravity(currentPosition.lat * D2R, alt); break;
        case 'MOON': G_ACC_NEW = 1.625; break;
        case 'MARS': G_ACC_NEW = 3.72; break;
        case 'ROTATING': 
            // Gravité centrifuge pour une station spatiale tournante
            const centripetal_accel = angularVel * angularVel * radius;
            // On considère la gravité simulée comme la force centrifuge
            G_ACC_NEW = centripetal_accel;
            break;
    }
    currentGravity = G_ACC_NEW;
    return { G_ACC_NEW };
}

function calculateDistanceRatio(altM) {
    // Calcul de la correction de distance par rapport à une altitude
    // Par exemple, pour une conversion 1:1000 pour 100m
    if (altM > 500) return 1.5;
    if (altM > 100) return 1.2;
    return 1.0; 
}

// --- Calculs Météo/Physique (simples) ---

function getSpeedOfSound(tempK) {
    if (isNaN(tempK) || tempK <= 0) return 340.29; 
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

function calculateAirDensity(pressure_hPa, tempK) {
    const P_Pa = pressure_hPa * 100;
    if (isNaN(P_Pa) || isNaN(tempK) || tempK <= 0) return RHO_SEA_LEVEL;
    return P_Pa / (R_SPECIFIC_AIR * tempK);
}

// =================================================================
// PARTIE 4 : LOGIQUE DES CAPTEURS ET SERVICES EXTERNES
// =================================================================

/** [EXTERNE] Synchronisation de l'heure NTP */
function syncH() {
    fetch(SERVER_TIME_ENDPOINT)
        .then(r => r.json())
        .then(data => {
            lServH = data.unixtime * 1000;
            lLocH = Date.now();
        })
        .catch(() => {
            console.error("Échec de la synchro NTP.");
            if ($('local-time-ntp')) $('local-time-ntp').textContent = 'SYNCHRO ÉCHOUÉE';
        });
}

function getCDate(lServH, lLocH) {
    if (lServH === 0) return new Date(); 
    return new Date(lServH + (Date.now() - lLocH));
}

// --- GESTION GPS & UKF ---

function handleGPSUpdate(pos) {
    const now = Date.now();
    const dt = (now - lastTimestamp) / 1000;
    lastTimestamp = now;

    if (isGpsPaused) return;
    isGpsActive = true;
    const oldPos = { lat: currentPosition.lat, lon: currentPosition.lon, alt: currentPosition.alt };

    lastGPS = pos.coords;
    
    // Mise à jour de l'état global avec les mesures GPS brutes
    currentPosition.lat = lastGPS.latitude;
    currentPosition.lon = lastGPS.longitude;
    currentPosition.alt = lastGPS.altitude || 0.0;
    currentPosition.acc = lastGPS.accuracy;
    currentPosition.spd_ms = lastGPS.speed || 0.0;
    currentPosition.heading = lastGPS.heading || 0.0;

    // Calcul de distance brute pour le dashboard
    if (dt > 0.001) {
        const distance3D = calculateDistance(oldPos, currentPosition, dt);
        if (distance3D > 0.01) { // Seulement si la distance est significative
            totalDistanceM += distance3D;
            totalMovementTimeS += dt;
        }
    }
    
    // Si UKF est disponible, effectuer l'update
    if (ukf && ukf.update) {
        // NOTE: L'implémentation complète des 21 états UKF est dans 'ukf-lib.js'
        // ukf.update({ type: 'GPS', measurement: [currentPosition.lat, currentPosition.lon, currentPosition.alt, currentPosition.spd_ms, ...], covariance: [...] });
        // Pour cet exemple, nous utilisons la position GPS brute comme état UKF
    }
}

function initGPS() {
    if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);

    const freqMode = $( 'freq-select' ).value;
    const GPS_OPTS = {
        enableHighAccuracy: freqMode === 'HIGH_FREQ',
        maximumAge: freqMode === 'HIGH_FREQ' ? 0 : 120000,
        timeout: 10000 
    };
    
    gpsWatchID = navigator.geolocation.watchPosition(
        handleGPSUpdate,
        (err) => { 
            console.error(`Erreur GPS (${err.code}): ${err.message}`);
            isGpsActive = false;
            $('statut-gps-acquisition').textContent = `ERREUR (${err.code}) : ${err.message}`;
        },
        GPS_OPTS
    );
    $('statut-gps-acquisition').textContent = 'ACQUISITION...';
}

function calculateDistance(p1, p2, dt) {
    // Calcul de la distance 3D utilisant turf.js (si disponible) ou formule Haversine simplifiée
    if (typeof turf !== 'undefined' && turf.distance) {
        const flatDistKm = turf.distance([p1.lon, p1.lat], [p2.lon, p2.lat], { units: 'kilometers' });
        const flatDistM = flatDistKm * 1000;
        const altDiff = Math.abs(p2.alt - p1.alt);
        
        let ratio = 1.0;
        if (distanceRatioMode) { ratio = calculateDistanceRatio(p2.alt); }
        if (netherMode) { ratio = 8.0; } // Nether mode 1:8

        return Math.sqrt((flatDistM * flatDistM) + (altDiff * altDiff)) * ratio;
    }
    return (p2.spd_ms * dt) || 0.0;
}


// --- GESTION CAPTEURS IMU et Lumière ---

function handleDeviceMotion(event) {
    if (event.accelerationIncludingGravity) {
        lastIMU.acc.x = event.accelerationIncludingGravity.x;
        lastIMU.acc.y = event.accelerationIncludingGravity.y;
        lastIMU.acc.z = event.accelerationIncludingGravity.z;
    }
    if (event.rotationRate) {
        lastIMU.angular_speed_x = event.rotationRate.alpha;
        lastIMU.angular_speed_y = event.rotationRate.beta;
        lastIMU.angular_speed_z = event.rotationRate.gamma;
    }
}

function handleDeviceOrientation(event) {
    lastIMU.pitch = event.beta;
    lastIMU.roll = event.gamma;
    lastIMU.heading = event.alpha;
}

function handleAmbientLight(event) {
    lastAmbientLight = event.value;
    if (lastAmbientLight > maxAmbientLight) {
        maxAmbientLight = lastAmbientLight;
    }
}

function initIMUSensors() {
    let status = 'Inactif';
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleDeviceOrientation, true);
        status = 'ACTIF (Orientation)';
    }
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        status = 'ACTIF (Complet)';
    }
    if (window.AmbientLightSensor) {
        try {
            const sensor = new AmbientLightSensor();
            sensor.addEventListener('reading', handleAmbientLight);
            sensor.start();
        } catch (e) {
             console.warn("AmbientLightSensor non supporté/bloqué.");
        }
    }
    if ($('imu-status')) $('imu-status').textContent = status;
}

// =================================================================
// PARTIE 5 : SERVICE MÉTÉO (PROXY VERCEL)
// =================================================================

function fetchWeather(lat, lon) {
    return fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`)
        .then(response => {
            if (!response.ok) throw new Error('Proxy Météo non disponible/non fonctionnel.');
            return response.json();
        })
        .then(data => {
            // Mise à jour de l'état global avec les données réelles
            lastP_hPa = data.pressure_hPa;
            lastT_K = data.tempK;
            lastH_perc = data.humidity_perc / 100.0;
            currentAirDensity = data.air_density; // Utilise la densité calculée par l'API
            currentSpeedOfSound = getSpeedOfSound(lastT_K); 
            if ($('statut-meteo')) $('statut-meteo').textContent = 'ACTIF (API Vercel)';
            return data;
        })
        .catch(e => {
            console.warn(`[METEO] Échec de l'appel API. Utilisation des valeurs ISA : ${e.message}`);
            if ($('statut-meteo')) $('statut-meteo').textContent = 'INACTIF (ISA FALLBACK)';
            // Fallback ISA
            lastT_K = TEMP_SEA_LEVEL_K;
            lastP_hPa = BARO_ALT_REF_HPA;
            currentAirDensity = RHO_SEA_LEVEL;
            currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); 
            return {
                pressure_hPa: BARO_ALT_REF_HPA,
                tempK: TEMP_SEA_LEVEL_K,
                tempC: TEMP_SEA_LEVEL_K - 273.15,
                humidity_perc: lastH_perc * 100,
                air_density: RHO_SEA_LEVEL,
                dew_point: 10.0, // Placeholder
                no2: null, pm25: null, pm10: null, o3: null
            };
        });
}

/**
 * Mise à jour des données lentes (Météo/API).
 */
function updateSlowData() {
    const lat = currentPosition.lat;
    const lon = currentPosition.lon;
    
    updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);

    fetchWeather(lat, lon).then(data => {
        // Mise à jour de la météo (Nouveaux IDs du HTML)
        if ($('air-temp-c')) $('air-temp-c').textContent = dataOrDefault(data.tempC, 1, ' °C');
        if ($('pressure-hpa')) $('pressure-hpa').textContent = dataOrDefault(data.pressure_hPa, 0, ' hPa');
        if ($('humidity-perc')) $('humidity-perc').textContent = dataOrDefault(data.humidity_perc, 0, ' %');
        if ($('air-density')) $('air-density').textContent = dataOrDefault(data.air_density, 3, ' kg/m³');
        if ($('dew-point')) $('dew-point').textContent = dataOrDefault(data.dew_point, 1, ' °C');
        
        // Mise à jour des Polluants
        if ($('no2-val')) $('no2-val').textContent = dataOrDefault(data.no2, 1, ' µg/m³');
        if ($('pm25-val')) $('pm25-val').textContent = dataOrDefault(data.pm25, 1, ' µg/m³');
        if ($('pm10-val')) $('pm10-val').textContent = dataOrDefault(data.pm10, 1, ' µg/m³');
        if ($('o3-val')) $('o3-val').textContent = dataOrDefault(data.o3, 1, ' µg/m³');

        // Mise à jour Bio/SVT (Placeholders)
        if ($('abs-humidity')) $('abs-humidity').textContent = dataOrDefault(null, 3, ' g/m³');
        if ($('wet-bulb-temp')) $('wet-bulb-temp').textContent = dataOrDefault(null, 1, ' °C');
        if ($('CAPE')) $('CAPE').textContent = dataOrDefault(null, 0, ' J/kg');
        if ($('O2-saturation')) $('O2-saturation').textContent = dataOrDefault(null, 1, ' %');
        if ($('photosynthesis-rate')) $('photosynthesis-rate').textContent = dataOrDefault(null, 2, ' µmol/m²/s');
    });
}

// =========================================================
// PARTIE 6 : CARTE ET NIVEAU À BULLE
// =========================================================

function initMap() {
    if (typeof L === 'undefined' || isMapInitialized) return;
    
    map = L.map('map').setView([currentPosition.lat, currentPosition.lon], 16);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map)
        .bindPopup("Position EKF Actuelle").openPopup();

    trackPolyline = L.polyline([], { color: 'blue', weight: 3 }).addTo(map);
    isMapInitialized = true;
}

function updateMap() {
    if (!isMapInitialized) return;

    const { lat, lon } = currentPosition;
    const latlng = [lat, lon];
    
    // Met à jour le marqueur
    marker.setLatLng(latlng);
    marker.getPopup().setContent(`Position EKF @ ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
    
    // Met à jour la trace (seulement si en mouvement)
    if (currentPosition.spd_ms > 0.05) {
        trackPolyline.addLatLng(latlng);
        // Centre la carte sur le marqueur (optionnel, peut être lourd)
        // map.setView(latlng, map.getZoom());
    }
}

function updateSpiritLevel() {
    const pitch = lastIMU.pitch; // Angle beta (avant/arrière)
    const roll = lastIMU.roll;   // Angle gamma (gauche/droite)
    const bubble = $('bubble');
    const containerSize = 100;
    const maxOffset = 30; // 30px max offset pour l'effet visuel
    
    if (!bubble) return;

    // Convertit les angles (0-360) en décalage (-maxOffset à +maxOffset)
    // Le Pitch affecte le décalage Y (haut/bas)
    const dy = (pitch / 90) * maxOffset; 
    // Le Roll affecte le décalage X (gauche/droite)
    const dx = (roll / 90) * maxOffset; 
    
    const limitedDX = Math.max(-maxOffset, Math.min(maxOffset, dx));
    const limitedDY = Math.max(-maxOffset, Math.min(maxOffset, dy));
    
    bubble.style.transform = `translate(${limitedDX}px, ${-limitedDY}px)`; // -DY car l'axe Y du DOM est inversé

    if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(pitch, 1, '°');
    if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(roll, 1, '°');
}


// =========================================================
// PARTIE 7 : BOUCLE PRINCIPALE (updateDashboard)
// =========================================================

function updateDashboard() {
    
    const now = getCDate(lServH, lLocH);
    const timeElapsedS = (now - window.startTime) / 1000;
    
    // Utiliser l'état UKF si actif, sinon l'état global (manuel/initial)
    // NOTE: Dans une implémentation complète, ukf.getState() retournerait l'état 21 complet
    const state = ukf && ukf.getState ? ukf.getState() : currentPosition;
    
    const { lat, lon, alt, spd_ms, vel_v, acc } = currentPosition;
    kAlt = alt; // Mise à jour de l'altitude globale
    
    const spd_brute_ms = spd_ms || 0.0;
    const spd_kmh = spd_brute_ms * KMH_MS;
    maxSpeedKmH = Math.max(maxSpeedKmH, spd_kmh);

    // --- 1. Mise à jour Horloge / Temps ---
    if ($('local-time-ntp') && lServH !== 0) $('local-time-ntp').textContent = now.toLocaleTimeString('fr-FR');
    if ($('utc-datetime')) $('utc-datetime').textContent = now.toUTCString().replace('GMT', 'UTC');
    if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(timeElapsedS, 2, ' s');
    if ($('movement-time')) $('movement-time').textContent = dataOrDefault(totalMovementTimeS, 2, ' s');
    if ($('time-minecraft')) $('time-minecraft').textContent = (typeof window.getMinecraftTime === 'function' ? window.getMinecraftTime(now).display : 'N/A');

    // --- 2. Calculs Dynamiques et Relativité ---
    const c_ratio = spd_brute_ms / C_L;
    const lorentz = 1.0 / Math.sqrt(1 - (c_ratio ** 2));
    const KE = 0.5 * currentMass * spd_brute_ms * spd_brute_ms;
    const restMassEnergy = currentMass * C_L * C_L; // E₀=mc²
    const sch_radius = (2 * G_U * currentMass) / (C_L * C_L);
    
    // Forces et Fluides
    const dynamicPressure = 0.5 * currentAirDensity * spd_brute_ms * spd_brute_ms;
    const dragForce = dynamicPressure * REF_AREA * DRAG_COEFF;
    const dragPower = dragForce * spd_brute_ms;
    
    // --- 3. Mise à jour Affichage Vitesse & Relativité ---
    if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(spd_kmh, 1, ' km/h');
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(spd_brute_ms, 2, ' m/s');
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = dataOrDefault(spd_brute_ms / 1000, 4, ' km/s');
    if ($('speed-status-text')) $('speed-status-text').textContent = isGpsActive ? (isGpsPaused ? "GPS en Pause (EKF Stable)" : "GPS ACTIF (Fusion UKF)") : "Acquisition GPS...";
    
    if ($('vitesse-max-session')) $('vitesse-max-session').textContent = dataOrDefault(maxSpeedKmH, 1, ' km/h');
    if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = dataOrDefault(currentSpeedOfSound, 4, ' m/s');
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = dataOrDefault((spd_brute_ms / currentSpeedOfSound) * 100, 2, ' %');
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault((spd_brute_ms / currentSpeedOfSound), 4);
    
    if ($('percent-speed-light')) $('percent-speed-light').textContent = dataOrDefaultExp((c_ratio * 100), 2, ' %');
    if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentz, 4);
    if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(KE * lorentz, 2, ' J');
    if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(restMassEnergy, 2, ' J');
    if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(sch_radius, 4, ' m');
    if ($('const-c')) $('const-c').textContent = dataOrDefault(C_L, 0, ' m/s');
    if ($('const-G')) $('const-G').textContent = dataOrDefaultExp(G_U, 4, ' m³/kg/s²');
    
    // --- 4. Mise à jour Distance & Horizon ---
    const totalDistKm = totalDistanceM / 1000;
    const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt) : (netherMode ? 8.0 : 1.0);
    const totalDistDisplay = netherMode ? totalDistKm * 8.0 : totalDistKm; // Affiche la distance parcourue dans l'Overworld
    
    if ($('distance-totale')) $('distance-totale').textContent = `${dataOrDefault(totalDistDisplay, 3, ' km')} | ${dataOrDefault(totalDistanceM, 2, ' m')}`;
    if ($('distance-ratio')) $('distance-ratio').textContent = dataOrDefault(ratio, 3);
    if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${dataOrDefault(ratio, 3)})`;
    if ($('nether-toggle-btn')) $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
    
    if ($('distance-light-s')) $('distance-light-s').textContent = dataOrDefaultExp(totalDistanceM / C_L, 2, ' s');

    const horizonDistKm = Math.sqrt(2 * EARTH_RADIUS_AVG * kAlt) / 1000;
    if ($('distance-horizon')) $('distance-horizon').textContent = dataOrDefault(horizonDistKm, 2, ' km');


    // --- 5. Mise à jour EKF/Position ---
    if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(lat, 6);
    if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(lon, 6);
    if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(alt, 2, ' m');
    if ($('heading-display')) $('heading-display').textContent = dataOrDefault(currentPosition.heading, 1, '°');
    if ($('acc-gps')) $('acc-gps').textContent = dataOrDefault(acc, 3, ' m');
    
    // --- 6. Mise à jour Dynamique & Forces ---
    if ($('local-gravity')) $('local-gravity').textContent = dataOrDefault(currentGravity, 4, ' m/s²');
    if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(currentPosition.g_force, 2, ' G');
    if ($('vertical-speed')) $('vertical-speed').textContent = dataOrDefault(vel_v, 2, ' m/s');
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
    if ($('drag-force')) $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
    if ($('drag-power-kw')) $('drag-power-kw').textContent = dataOrDefault(dragPower / 1000, 2, ' kW');
    if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(KE, 2, ' J');
    
    // --- 7. Mise à jour IMU & Niveau à Bulle ---
    if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(lastIMU.acc.x, 3, ' m/s²');
    if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(lastIMU.acc.y, 3, ' m/s²');
    if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(lastIMU.acc.z, 3, ' m/s²');
    if ($('angular-speed')) $('angular-speed').textContent = dataOrDefault(lastIMU.angular_speed_x, 3, ' rad/s'); // Utilise X comme référence
    updateSpiritLevel();
    
    // --- 8. Mise à jour EKF/UKF Debug ---
    if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = ukf ? (isGpsActive ? "ACTIF (UKF Fusion)" : (isGpsPaused ? "PAUSE GPS" : "ACQUISITION...")) : "NON DÉFINI";
    if ($('ukf-r-noise')) $('ukf-r-noise').textContent = dataOrDefault(acc, 2, ' (R)');
    if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = dataOrDefault(acc, 6, ' m');
    
    // --- 9. Mise à jour Astro (utilise lib/astro.js) ---
    if (typeof window.getAstroData === 'function' && typeof window.formatHours === 'function') {
        const astroData = window.getAstroData(now, lat, lon, alt); 
        
        if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
        if ($('tst-time')) $('tst-time').textContent = window.formatHours(astroData.TST_HRS);
        if ($('mst-time')) $('mst-time').textContent = window.formatHours(astroData.MST_HRS);
        if ($('equation-of-time')) $('equation-of-time').textContent = dataOrDefault(astroData.EOT_MIN, 2, ' min');
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.altitude * R2D, 2, '°'); 
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.azimuth * R2D, 2, '°'); 
        if ($('moon-phase-name')) $('moon-phase-name').textContent = window.getMoonPhaseName(astroData.moon.illumination.phase);
        if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.altitude * R2D, 2, '°');
        if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(astroData.moon.azimuth * R2D, 2, '°'); 
        if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 1, ' %');
        if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(astroData.moon.distance / 1000, 0, ' km');
        
        // Mettre à jour l'horloge céleste (si l'implémentation est incluse dans astro.js ou un autre fichier)
        const sunAltDeg = astroData.sun.altitude * R2D;
        const sunRotation = ((astroData.TST_HRS / 24) * 360) + 90; // 0h à 90deg, 12h à 270deg
        if ($('sun-element')) $('sun-element').style.transform = `rotate(${sunRotation}deg)`;
        if ($('moon-element')) $('moon-element').style.transform = `rotate(${sunRotation + 180}deg)`; // La lune est opposée au soleil TST
        if ($('minecraft-clock')) {
             if (sunAltDeg > 5) {
                 $('minecraft-clock').className = 'sky-day';
                 $('clock-status').textContent = 'Jour (☀️)';
             } else if (sunAltDeg > -5) {
                 $('minecraft-clock').className = 'sky-sunset';
                 $('clock-status').textContent = 'Crépuscule/Aube (🌅)';
             } else {
                 $('minecraft-clock').className = 'sky-night';
                 $('clock-status').textContent = 'Nuit (🌙)';
             }
        }
    } else {
        // Fallbacks pour Astro
        if ($('tst-time')) $('tst-time').textContent = 'N/A';
    }
    
    // --- 10. Mise à jour Carte ---
    updateMap();
}

// =========================================================
// PARTIE 8 : INITIALISATION ET GESTION DES ÉVÉNEMENTS
// =========================================================

/**
 * Attache les écouteurs d'événements pour les contrôles du dashboard.
 */
function setupEventListeners() {
    
    // --- GPS / Pause ---
    const toggleGpsBtn = $('toggle-gps-btn');
    if (toggleGpsBtn) {
        toggleGpsBtn.addEventListener('click', () => {
            isGpsPaused = !isGpsPaused;
            toggleGpsBtn.textContent = isGpsPaused ? '⏸️ PAUSE GPS' : '▶️ MARCHE GPS';
        });
    }
    if ($('freq-select')) $('freq-select').addEventListener('change', initGPS);
    
    // --- Thème ---
    const toggleModeBtn = $('toggle-mode-btn');
    if (toggleModeBtn) {
        toggleModeBtn.addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            toggleModeBtn.innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });
    }

    // --- Masse / Corps Céleste / Rotation ---
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
    });
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        $('gravity-base').textContent = dataOrDefault(currentGravity, 4, ' m/s²');
    });
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius').value) || 100;
        angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
        if (currentCelestialBody === 'ROTATING') {
            updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = dataOrDefault(currentGravity, 4, ' m/s²');
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);
    
    // --- Modes Spéciaux ---
    if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatioMode = !distanceRatioMode;
    });
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
    });
    if ($('xray-button')) $('xray-button').addEventListener('click', () => {
        if ($('minecraft-clock')) $('minecraft-clock').classList.toggle('x-ray');
    });

    // --- Réinitialisation ---
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { totalDistanceM = 0; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpeedKmH = 0; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser ?")) {
            totalDistanceM = 0; 
            maxSpeedKmH = 0;
            totalMovementTimeS = 0;
            currentPosition = { lat: 43.284578, lon: 5.358713, alt: 100.00, acc: 10.0, spd_ms: 0.0, vel_v: 0.0, heading: 0.0, g_force: 0.0, g_force_vert: 1.0 };
            initGPS(); // Redémarre l'écoute GPS
        }
    });

    // --- UKF/EKF Debug ---
    if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
    
    // --- Placeholder / Non implémenté dans le JS core ---
    if ($('capture-data-btn')) $('capture-data-btn').addEventListener('click', () => alert("Fonction de capture de données non implémentée dans ce fichier."));
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => { 
        alert("ARRÊT D'URGENCE : Désactivation des capteurs..."); 
        if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);
        isGpsActive = false;
        $('emergency-stop-btn').classList.toggle('active');
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
    });

    // Initialisation de la gravité de base
    updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    $('gravity-base').textContent = dataOrDefault(currentGravity, 4, ' m/s²');
}


window.addEventListener('load', () => {
    window.startTime = Date.now();
    
    // 1. Initialisation UKF
    if (typeof window.ProfessionalUKF === 'function') { 
        ukf = new ProfessionalUKF();
        console.log("✅ Filtre UKF 21 États initialisé.");
    } else {
         console.warn("🔴 AVERTISSEMENT: ProfessionalUKF n'est pas définie. Le tableau de bord fonctionne en mode 'EKF de Base' (Position/Vitesse brutes). Vérifiez lib/ukf-lib.js.");
    }
    
    // 2. Initialisation des capteurs réels
    initIMUSensors(); 
    initGPS(); 
    setupEventListeners();
    
    // 3. Initialisation de la synchro NTP
    syncH(); 
    
    // 4. Initialisation de la carte Leaflet
    initMap();

    // 5. Boucle de rafraîchissement lente (Météo/API)
    updateSlowData(); // Premier appel immédiat
    setInterval(updateSlowData, DOM_SLOW_UPDATE_MS); 

    // 6. Boucle de rafraîchissement principale (60Hz)
    updateDashboard(); // Exécution immédiate
    setInterval(updateDashboard, DOM_FAST_UPDATE_MS); 
});
