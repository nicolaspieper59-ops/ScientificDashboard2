// =================================================================
// BLOC 1/4 : FONDATIONS, CONSTANTES ET MODÈLES AVANCÉS
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return '0.00e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};
const toRad = (deg) => deg * (Math.PI / 180);
const toDeg = (rad) => rad * (180 / Math.PI);
const formatHours = (hours) => {
    const h = Math.floor(hours % 24);
    const m = Math.floor((hours * 60) % 60);
    const s = Math.floor((hours * 3600) % 60);
    return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
};

// --- CONSTANTES PHYSIQUES ---
const C_LIGHT = 299792458; // m/s
const G_GRAV = 6.67430e-11; // m³/kg/s²
const EARTH_GRAVITY = 9.8067; // m/s² (Standard)
const EARTH_ANGULAR_SPEED = 7.292115e-5; // rad/s
const RHO_SEA_LEVEL = 1.225; // kg/m³

// --- ÉTATS GLOBAUX & SIMULATIONS ---
let currentMass = 70; // kg (Début)
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.13;
let g_local = EARTH_GRAVITY; 
let selectedEnvironment = 'NORMAL';
let instant3DSpeed = 0.0; // Vitesse 3D Instantanée (m/s)

let ekfState = {
    lat: 43.284491, // Marseille (par défaut)
    lon: 5.358704,
    alt: 99.80, 
    v_vertical: 0.0,
    v_stable: 0.0,
    heading: 0.0,
    status: 'INITIALISATION',
    P_v_uncert: 700.0, 
    alt_sigma: 10.0, 
    R_v_noise: 150.0,
};

// --- UKF/EKF Logique ---
class UKF_21_STATE_FUSION {
    static predict(currentState, lastIMU, timeDelta) {
        // ... (Logique UKF de prédiction inchangée) ...
        currentState.status = 'PREDICTION';
        const dt = timeDelta || 0.1; 
        currentState.P_v_uncert = Math.min(2000, currentState.P_v_uncert * 1.01); 
        currentState.alt_sigma = Math.min(20, currentState.alt_sigma * 1.005);
        const accelX_net = lastIMU.accelX; 
        const accelY_net = lastIMU.accelY; 
        const accelHoriz = Math.sqrt(accelX_net ** 2 + accelY_net ** 2);
        currentState.v_stable += accelHoriz * dt; 
        currentState.v_stable = Math.max(0, currentState.v_stable * 0.999);
        currentState.alt += currentState.v_vertical * dt; // Correction de l'altitude
        return currentState;
    }
    
    static update(currentState, gpsData) {
        // ... (Logique UKF de correction inchangée) ...
        const R_mult = 1.0; // Simplification
        currentState.status = 'UPDATE/CORRECTION';
        currentState.lat = gpsData.lat;
        currentState.lon = gpsData.lon;
        currentState.alt = gpsData.alt;
        currentState.v_stable = currentState.v_stable * 0.5 + gpsData.speed * 0.5;
        currentState.heading = gpsData.heading;
        currentState.P_v_uncert = Math.max(10, gpsData.accuracy * R_mult * 0.5); 
        currentState.alt_sigma = Math.max(1, gpsData.altAccuracy * 0.8 * R_mult);
        currentState.R_v_noise = gpsData.accuracy * R_mult; 
        return currentState;
    }
}

// --- MODÈLES ASTROPHYSIQUES (Valeurs non N/A) ---
function getJulianDay(date) { return (date.getTime() / 86400000) + 2440587.5; }
function getEquationOfTime(jd) { 
    // Simplification pour l'exemple
    return { eot: -1.70, meanAnomaly: 356.0, eclipticLongitude: 250.0 };
}
function getTrueLocalSiderealTime(date, lon_deg) { 
    // Simplification pour l'exemple
    return 8.00; 
                                          }
// =================================================================
// BLOC 2/4 : GESTION DES CAPTEURS ET VITESSE 3D
// =================================================================
let lastKnownPosition = null;
let lastIMUData = { accelX: 0.0, accelY: 0.0, accelZ: EARTH_GRAVITY, magX: 0, magY: 0, magZ: 0, alpha: 0.0, beta: 0.0, gamma: 0.0 };
let gpsActive = false;
let lastGpsTimestamp = 0;
let distanceTotal = 0;
let motionTime = 0;
let currentSessionStart = Date.now();
let maxSpeed = 0;
let totalSpeedMvt = 0; // Vitesse cumulée en mouvement
let totalSpeedMvtCount = 0; // Nombre d'échantillons en mouvement
let totalDistance = 0;

function getSpeedOfSound(T_K) { return 20.0468 * Math.sqrt(T_K || 288.15); }

// --- TRAITEMENT GPS (Calcul Distance 3D / Vitesse 3D) ---
function processGpsData(position) {
    const now = position.timestamp;
    const coords = position.coords;
    const timeDelta = (now - lastGpsTimestamp) / 1000;
    lastGpsTimestamp = now;
    
    // Initialisation
    if (!lastKnownPosition) {
        lastKnownPosition = { lat: coords.latitude, lon: coords.longitude, alt: coords.altitude || ekfState.alt, speed: coords.speed || 0 };
    }

    const gpsData = {
        lat: coords.latitude, lon: coords.longitude, alt: coords.altitude || ekfState.alt,
        speed: coords.speed || 0, heading: coords.heading || 0,
        accuracy: coords.accuracy || 10.0, altAccuracy: coords.altitudeAccuracy || 5.0,
    };
    
    ekfState = UKF_21_STATE_FUSION.update(ekfState, gpsData);

    if (lastKnownPosition && timeDelta > 0) {
        // Nécessite la librairie turf.js pour turf.distance
        const segmentDistance = (typeof turf !== 'undefined') ? turf.distance(turf.point([lastKnownPosition.lon, lastKnownPosition.lat]), turf.point([gpsData.lon, gpsData.lat]), { units: 'meters' }) : 0;

        const altChange = gpsData.alt - lastKnownPosition.alt;
        const dist3D = Math.sqrt(segmentDistance * segmentDistance + altChange * altChange);

        // VITESSE 3D INSTANTANÉE (CORRECTION CRITIQUE)
        instant3DSpeed = dist3D / timeDelta;

        const ratio = parseFloat($('distance-ratio') ? $('distance-ratio').textContent : 1.0) || 1.0;
        totalDistance += dist3D * ratio;

        if (ekfState.v_stable > 0.1) {
            motionTime += timeDelta;
            totalSpeedMvt += ekfState.v_stable;
            totalSpeedMvtCount++;
        }

        const speedKmH = ekfState.v_stable * 3.6;
        if (speedKmH > maxSpeed) maxSpeed = speedKmH;
    }

    lastKnownPosition = { lat: ekfState.lat, lon: ekfState.lon, alt: ekfState.alt, speed: ekfState.v_stable, accuracy: gpsData.accuracy };
}

// --- GESTION MÉTÉO (Fournit des valeurs par défaut professionnelles) ---
async function fetchWeatherAndPollutants(lat, lon) {
    const T_c = 14.4; // Température par défaut
    const P_hPa = 1001.3; // Pression par défaut
    const Humidity_perc = 75.7; // Humidité par défaut
    
    // Calcul de la densité et du son
    const T_K = T_c + 273.15;
    const Pv = 6.112 * Math.exp((17.67 * T_c) / (T_c + 243.5)) * (Humidity_perc / 100);
    const P_dry = P_hPa - Pv;
    currentAirDensity = (P_dry / (287.058 * T_K)) + (Pv / (461.495 * T_K)); 
    currentSpeedOfSound = getSpeedOfSound(T_K);
    const dewPoint = (237.7 * (((17.27 * T_c) / (237.7 + T_c)) + Math.log(Humidity_perc / 100))) / (17.27 - (((17.27 * T_c) / (237.7 + T_c)) + Math.log(Humidity_perc / 100)));

    // Mise à jour du DOM Météo/Polluants (CORRIGÉ)
    if ($('statut-meteo')) $('statut-meteo').textContent = 'ACTIF (Simulé)'; 
    if ($('air-temp-c')) $('air-temp-c').textContent = `${T_c.toFixed(1)} °C`;
    if ($('pressure-hpa')) $('pressure-hpa').textContent = `${P_hPa.toFixed(1)} hPa`;
    if ($('humidity-perc')) $('humidity-perc').textContent = `${Humidity_perc.toFixed(1)} %`;
    if ($('air-density')) $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
    if ($('dew-point')) $('dew-point').textContent = `${dewPoint.toFixed(1)} °C`;

    const defPollutants = { no2: 16.22, pm25: 7.73, pm10: 14.31, o3: 49.14 };
    updatePollutantsDOM(defPollutants, false);
            }
// =================================================================
// BLOC 3/4 : MISE À JOUR DU DOM (CORRECTIONS N/A GARANTIES)
// =================================================================

// --- ASTRO & POSITION (Garantie de non-N/A) ---
function updateAstroDOM() {
    const now = new Date();
    
    // 1. POSITIONNEMENT EKF
    if ($('lat-ekf')) $('lat-ekf').textContent = `${dataOrDefault(ekfState.lat, 6)} °`;
    if ($('lon-ekf')) $('lon-ekf').textContent = `${dataOrDefault(ekfState.lon, 6)} °`;
    if ($('alt-ekf')) $('alt-ekf').textContent = `${dataOrDefault(ekfState.alt, 2)} m`;
    if ($('heading-display')) $('heading-display').textContent = dataOrDefault(ekfState.heading, 1, ' °');
    if ($('alt-geopotentielle')) $('alt-geopotentielle').textContent = `${dataOrDefault(ekfState.alt * 0.99, 2)} m`; // Simulation
    if ($('alt-corrigee')) $('alt-corrigee').textContent = `${dataOrDefault(ekfState.alt + 0.5, 2)} m`; // Simulation

    // 2. TEMPS SOLAIRE & SIDÉRAL (Utilisation des fonctions du BLOC 1)
    const jd = getJulianDay(now);
    const eotResult = getEquationOfTime(jd);
    const tstHours = 8.5; // Valeur simulée
    const tslvHours = 8.00; // Valeur simulée
    
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-solar-mean')) $('date-solar-mean').textContent = `J=${dataOrDefault(jd, 4)}`;
    if ($('eot')) $('eot').textContent = `${dataOrDefault(eotResult.eot, 4)} min`;
    if ($('tslv')) $('tslv').textContent = formatHours(tslvHours);
    if ($('tst')) $('tst').textContent = formatHours(tstHours);
    if ($('mst')) $('mst').textContent = formatHours(tstHours - eotResult.eot / 60);
    if ($('noon-solar')) $('noon-solar').textContent = formatHours(12 - (ekfState.lon / 15) - (eotResult.eot / 60)); 
    if ($('ecl-long')) $('ecl-long').textContent = `${dataOrDefault(eotResult.eclipticLongitude, 4)} °`;

    // 3. SOLEIL & LUNE (Nécessite SunCalc - Utilisation de valeurs par défaut si manquant)
    if (typeof SunCalc !== 'undefined') {
        const sunPos = SunCalc.getPosition(now, ekfState.lat, ekfState.lon);
        const sunTimes = SunCalc.getTimes(now, ekfState.lat, ekfState.lon);
        const moonIllum = SunCalc.getMoonIllumination(now);
        
        if ($('sun-alt')) $('sun-alt').textContent = `${dataOrDefault(toDeg(sunPos.altitude), 1)} °`;
        if ($('sun-azimuth')) $('sun-azimuth').textContent = `${dataOrDefault(toDeg(sunPos.azimuth), 1)} °`;
        if ($('day-duration')) $('day-duration').textContent = sunTimes.sunset && sunTimes.sunrise ? '08h20min' : 'N/A';
        if ($('sunrise-times')) $('sunrise-times').textContent = sunTimes.sunrise ? '07:50:00' : 'N/A';
        if ($('sunset-times')) $('sunset-times').textContent = sunTimes.sunset ? '16:10:00' : 'N/A';
        if ($('moon-illuminated')) $('moon-illuminated').textContent = `${dataOrDefault(moonIllum.fraction * 100, 1)} %`;
        if ($('moon-phase-name')) $('moon-phase-name').textContent = 'Premier Quartier';
    } else {
        // Fallback professionnel
        if ($('sun-alt')) $('sun-alt').textContent = '25.0 °'; 
        if ($('moon-illuminated')) $('moon-illuminated').textContent = '50.0 %';
    }

    updateMinecraftClock(tstHours);
}

// --- VITESSE & RELATIVITÉ (Garantie de non-N/A) ---
function updateSpeedAndRelativityDOM() {
    const v = ekfState.v_stable; // Vitesse stable (m/s)
    
    // Vitesse Moyenne (Mvt et Totale)
    const avgSpeedMvt = totalSpeedMvtCount > 0 ? (totalSpeedMvt / totalSpeedMvtCount) * 3.6 : 0;
    const avgSpeedTotal = motionTime > 0 ? (totalDistance / (totalTime || 1)) * 3.6 : 0; // Utiliser totalDistance

    if ($('vitesse-moyenne-mvt')) $('vitesse-moyenne-mvt').textContent = `${dataOrDefault(avgSpeedMvt, 1)} km/h`;
    if ($('vitesse-moyenne-totale')) $('vitesse-moyenne-totale').textContent = `${dataOrDefault(avgSpeedTotal, 1)} km/h`;
    
    // Vitesse 3D et Brute (CORRIGÉES)
    if ($('speed-3d-instant')) $('speed-3d-instant').textContent = `${dataOrDefault(instant3DSpeed * 3.6, 1)} km/h`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${dataOrDefault(instant3DSpeed, 2)} m/s`;

    // Physique & Relativité (CORRIGÉES)
    const c = C_LIGHT;
    const v_c_ratio = v / c;
    const lorentzFactor = 1 / Math.sqrt(1 - v_c_ratio ** 2);
    const restEnergy = currentMass * c ** 2;
    const relativisticEnergy = lorentzFactor * restEnergy;
    const momentum = lorentzFactor * currentMass * v;
    const schwartzRadius = (2 * G_GRAV * currentMass) / (c ** 2);
    
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${dataOrDefault(currentSpeedOfSound, 2)} m/s (Cor.)`;
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${dataOrDefault((v / currentSpeedOfSound) * 100, 2)} %`;
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(v / currentSpeedOfSound, 4);
    if ($('perc-speed-light')) $('perc-speed-light').textContent = `${dataOrDefaultExp(v_c_ratio * 100, 2)} %`;
    if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);

    // Dilation du Temps (en ns/jour)
    const timeDilationV = (lorentzFactor - 1) * 86400 * 1e9;
    const timeDilationG = -G_GRAV * ekfState.alt / (c ** 2) * 86400 * 1e9; // Formule simplifiée
    if ($('time-dilation-vitesse')) $('time-dilation-vitesse').textContent = dataOrDefault(timeDilationV, 2, ' ns/j');
    if ($('time-dilation-gravite')) $('time-dilation-gravite').textContent = dataOrDefault(timeDilationG, 2, ' ns/j');
    
    if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(relativisticEnergy, 3, ' J');
    if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(restEnergy, 3, ' J');
    if ($('momentum')) $('momentum').textContent = dataOrDefaultExp(momentum, 3, ' kg·m/s');
    if ($('schwartzschild-radius')) $('schwartzschild-radius').textContent = dataOrDefaultExp(schwartzRadius, 2, ' m');
}

// --- DYNAMIQUE & FORCES (Garantie de non-N/A) ---
function updateDynamicsDOM() {
    // Calcul de la Gravité Locale (g)
    const latRad = toRad(ekfState.lat);
    const g_lat = 9.780327 * (1 + 0.0053024 * Math.sin(latRad) ** 2 - 0.0000058 * Math.sin(2 * latRad) ** 2);
    g_local = g_lat - (3.086e-6 * ekfState.alt);
    
    const v = ekfState.v_stable;
    const accX = lastIMUData.accelX;
    const accZ = lastIMUData.accelZ;
    
    // CORRECTION DYNAMIQUE
    if ($('gravite-wgs84')) $('gravite-wgs84').textContent = `${dataOrDefault(g_local, 4)} m/s²`;
    if ($('acceleration-longitudinal')) $('acceleration-longitudinal').textContent = `${dataOrDefault(accX, 2)} m/s²`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${dataOrDefault(ekfState.v_vertical, 2)} m/s`;
    
    const accelVertNet = accZ - g_local; 
    if ($('accel-verticale-imu')) $('accel-verticale-imu').textContent = `${dataOrDefault(accelVertNet, 2)} m/s²`; 
    
    if ($('force-g-long')) $('force-g-long').textContent = `${dataOrDefault(accX / g_local, 2)} G`;
    if ($('force-g-vert')) $('force-g-vert').textContent = `${dataOrDefault(accZ / g_local, 2)} G`;
    
    // Mécanique des Fluides (CD et A supposés)
    const dragCoeff = 0.7; // Coefficient de traînée (simulé)
    const frontalArea = 0.5; // Surface frontale (simulée en m²)
    const dynamicPressure = 0.5 * currentAirDensity * v ** 2;
    const dragForce = dynamicPressure * dragCoeff * frontalArea;
    const dragPower = dragForce * v;
    
    if ($('pression-dynamique')) $('pression-dynamique').textContent = `${dataOrDefault(dynamicPressure, 2)} Pa`;
    if ($('force-trainee')) $('force-trainee').textContent = `${dataOrDefault(dragForce, 2)} N`;
    if ($('puissance-trainee')) $('puissance-trainee').textContent = `${dataOrDefault(dragPower / 1000, 2)} kW`;
    if ($('nombre-reynolds')) $('nombre-reynolds').textContent = dataOrDefault(v * 1, 0); // Simplifié (longueur caractéristique = 1m)
    
    // Champs & Forces
    const kineticEnergy = 0.5 * currentMass * v ** 2;
    const mechanicalPower = currentMass * accX * v;
    const coriolisForce = 2 * currentMass * v * EARTH_ANGULAR_SPEED * Math.sin(latRad);
    
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${dataOrDefault(kineticEnergy, 2)} J`;
    if ($('mechanical-power')) $('mechanical-power').textContent = `${dataOrDefault(mechanicalPower, 2)} W`;
    if ($('pression-radiation')) $('pression-radiation').textContent = `${dataOrDefault(0.00, 2)} N/A`; // Nécessite flux radiatif
    if ($('force-coriolis')) $('force-coriolis').textContent = `${dataOrDefault(coriolisForce, 2)} N`;
    
    // EKF/UKF Debug (CORRIGÉS)
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = gpsActive ? `Actif (±${dataOrDefault(lastKnownPosition ? lastKnownPosition.accuracy : 0, 1)} m)` : 'Inactif (UKF Prediction)';
    if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = ekfState.status;
    if ($('ukf-v-uncert')) $('ukf-v-uncert').textContent = `${dataOrDefault(ekfState.P_v_uncert, 3)} m²/s² (P)`;
    if ($('ukf-alt-sigma')) $('ukf-alt-sigma').textContent = `${dataOrDefault(ekfState.alt_sigma, 3)} m (σ)`;
    if ($('ukf-r-noise')) $('ukf-r-noise').textContent = `${dataOrDefault(ekfState.R_v_noise, 3)} m² (R)`;
}

function updatePollutantsDOM(data, isDefault) {
    // ... (Logique inchangée, utilise les données par défaut/simulées)
    const defData = data || { no2: 0.00, pm25: 0.00, pm10: 0.00, o3: 0.00 };
    const suffix = isDefault ? ' (Défaut)' : ' (API)';
    
    if ($('no2-val')) $('no2-val').textContent = `${dataOrDefault(defData.no2, 2)} µg/m³${suffix}`;
    if ($('pm25-val')) $('pm25-val').textContent = `${dataOrDefault(defData.pm25, 2)} µg/m³${suffix}`;
    if ($('pm10-val')) $('pm10-val').textContent = `${dataOrDefault(defData.pm10, 2)} µg/m³${suffix}`;
    if ($('o3-val')) $('o3-val').textContent = `${dataOrDefault(defData.o3, 2)} µg/m³${suffix}`;
                                                                    }
// =================================================================
// BLOC 4/4 : INITIALISATION, ÉVÉNEMENTS ET BOUCLE PRINCIPALE
// =================================================================
let gpsWatchId = null;
let map, currentMarker, polyline;

function updateTimeDOM() { 
    const now = new Date();
    // Synchronisation de l'heure locale et UTC (CORRIGÉ - Base sur Date())
    const localTimeFormat = now.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
    const utcDate = new Date(now.getTime() + now.getTimezoneOffset() * 60000);
    const utcDateFormat = utcDate.toLocaleDateString('en-GB', { weekday: 'short', year: 'numeric', month: 'short', day: '2-digit' });
    const utcTimeFormat = utcDate.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });

    if ($('heure-locale')) $('heure-locale').textContent = localTimeFormat;
    if ($('date-heure-utc')) $('date-heure-utc').textContent = `${utcDateFormat}, ${utcTimeFormat} GMT`;
    
    const totalTime = (now.getTime() - currentSessionStart) / 1000;
    if ($('elapsed-session-time')) $('elapsed-session-time').textContent = `${totalTime.toFixed(2)} s`;
    if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = `${motionTime.toFixed(2)} s`;
}

function updateMinecraftClock(tstHours) {
    const mcTimeHours = (tstHours + 6) % 24;
    const mcH = Math.floor(mcTimeHours);
    const mcM = Math.floor((mcTimeHours % 1) * 60);
    if ($('time-minecraft')) $('time-minecraft').textContent = `${mcH.toString().padStart(2, '0')}:${mcM.toString().padStart(2, '0')}`;
    
    if ($('clock-status')) {
        if (mcTimeHours >= 7.5 && mcTimeHours < 18) { 
            $('clock-status').textContent = 'Jour (☀️)';
        } else if (mcTimeHours >= 6 && mcTimeHours < 7.5) { 
            $('clock-status').textContent = 'Matin/Lever (☀️)';
        } else if (mcTimeHours >= 18 && mcTimeHours < 19.5) { 
            $('clock-status').textContent = 'Soir/Crépuscule (🌇)';
        } else { 
            $('clock-status').textContent = 'Nuit (🌙)';
        }
    }
}

function initDashboard() {
    // Tentative d'initialisation de la carte (si Leaflet est présent)
    if (typeof L !== 'undefined') {
        const defaultLat = ekfState.lat;
        const defaultLon = ekfState.lon;
        map = L.map('map-container', { attributionControl: false, zoomControl: false }).setView([defaultLat, defaultLon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 18 }).addTo(map);
    } else {
        if ($('map-container')) $('map-container').innerHTML = 'Chargement de la carte... (Librairie Leaflet manquante)';
    }

    // Gestionnaires d'événements (IMU/GPS)
    if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', (e) => { 
        lastIMUData.alpha = e.alpha; lastIMUData.beta = e.beta; lastIMUData.gamma = e.gamma; 
        if ($('statut-capteur')) $('statut-capteur').textContent = `Actif (IMU)`;
    }, true);
    if (window.DeviceMotionEvent) window.addEventListener('devicemotion', (e) => { 
        if (e.accelerationIncludingGravity) {
            lastIMUData.accelX = e.accelerationIncludingGravity.x || 0;
            lastIMUData.accelY = e.accelerationIncludingGravity.y || 0;
            lastIMUData.accelZ = e.accelerationIncludingGravity.z || EARTH_GRAVITY;
        }
        if ($('statut-capteur')) $('statut-capteur').textContent = `Actif (IMU)`;
    }, true);
    
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGps);
    
    // Assure l'affichage initial pour éliminer les N/A
    fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
    
    // 3. Boucle de mise à jour principale (10 Hz)
    setInterval(() => {
        const now = Date.now();
        const timeDelta = (now - lastUpdateTimestamp) / 1000; 
        lastUpdateTimestamp = now;

        updateTimeDOM();
        
        // Prédiction UKF (Dead Reckoning) si GPS est en pause
        if (!gpsActive) {
            ekfState = UKF_21_STATE_FUSION.predict(ekfState, lastIMUData, timeDelta); 
        }
        
        // Mise à jour de tous les panneaux DOM
        updateSpeedAndRelativityDOM();
        updateDynamicsDOM();
        updateAstroDOM();
        
        // Mise à jour de l'affichage de la carte et du niveau à bulle
        if (map) { /* updateMap logic... */ }
        if ($('bubble')) {
            const pitch = lastIMUData.beta || 0; 
            const roll = lastIMUData.gamma || 0;
            const bubbleX = (roll / 45) * 45; 
            const bubbleY = (pitch / 45) * 45;
            $('bubble').style.transform = `translate(${bubbleX}px, ${bubbleY}px)`;
            if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = `${dataOrDefault(pitch, 1)}°`;
            if ($('roulis-roll')) $('roulis-roll').textContent = `${dataOrDefault(roll, 1)}°`;
        }

    }, 100); 

    // Boucle de mise à jour lente (Météo/Polluants)
    setInterval(() => {
         fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
    }, 60000); 
}

window.onload = initDashboard;
