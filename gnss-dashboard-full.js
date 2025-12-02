// =================================================================
// BLOC 1/4 : CONSTANTES, UKF & MODÈLES PHYSIQUES
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
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};
const toRad = (deg) => deg * (Math.PI / 180);
const toDeg = (rad) => rad * (180 / Math.PI);
const formatTime = (date) => `${date.getHours().toString().padStart(2, '0')}:${date.getMinutes().toString().padStart(2, '0')}:${date.getSeconds().toString().padStart(2, '0')}`;
const EARTH_RADIUS = 6371000; // m

// --- CONSTANTES PHYSIQUES ET ÉTATS ---
const C_LIGHT = 299792458; // m/s
const G_GRAV = 6.67430e-11; // m³/kg/s²
const EARTH_GRAVITY = 9.8067; // m/s² (Standard)
const SPEED_SOUND_STP = 340.13; // m/s à 15°C
const TEMP_SEA_LEVEL_K = 288.15; // K (15°C)
const BARO_ALT_REF_HPA = 1013.25; // hPa

let currentMass = 70; // kg
let currentGravity = EARTH_GRAVITY;
let currentAirDensity = 1.213; // kg/m³ (à 14.4°C)
let currentSpeedOfSound = SPEED_SOUND_STP;

const ENVIRONMENT_FACTORS = {
    'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- MODÈLES PHYSIQUES ---
function getSpeedOfSound(T_K) {
    return 20.0468 * Math.sqrt(T_K); // m/s
}

// --- UKF/EKF SIMULÉ (États d'intérêt pour le DOM) ---
let ekfState = {
    lat: 43.284491,
    lon: 5.358704,
    alt: 99.80, // m
    v_vertical: 0.0,
    v_stable: 0.0,
    heading: 0.0,
    status: 'INITIALISATION',
    P_v_uncert: 675.503, // Incertitude vitesse (P)
    alt_sigma: 9.447, // Incertitude altitude (σ)
    R_v_noise: 142.206, // Bruit mesure R
};
let gpsActive = false;
let isMoving = false;


// --- CALCULS ASTRONOMIQUES AVANCÉS (Pour les champs N/A) ---

/**
 * Calcule le Jour Julien (Julian Day) pour une date UTC donnée.
 * @param {Date} date - Date UTC
 * @returns {number} Jour Julien
 */
function getJulianDay(date) {
    const time = date.getTime() / 86400000; // Millisecondes par jour
    const offset = 2440587.5; // JD de référence (1er jan 1970 00:00:00 UTC)
    return time + offset;
}

/**
 * Calcule l'Équation du Temps (EOT) en minutes.
 * Simplifié basé sur l'anomalie moyenne et la longitude écliptique.
 * @param {number} jd - Jour Julien
 * @returns {{eot: number, meanAnomaly: number, eclipticLongitude: number}}
 */
function getEquationOfTime(jd) {
    const T = (jd - 2451545.0) / 36525; // Temps écoulé en siècles juliens depuis J2000
    const w = toRad(282.9404 + 4.70935E-5 * T); // Longitude du périhélie
    const a = 1.0; // Demi-grand axe (UA)
    const e = 0.016709 - 1.151E-9 * T; // Excentricité de l'orbite
    const M = toRad(356.0470 + 35999.0503 * T); // Anomalie moyenne

    // Équation de Kepler pour obtenir la Vraie Anomalie (v) et la Longitude du Soleil (L)
    const E = M + e * Math.sin(M) * (1 + e * Math.cos(M));
    const L = w + E; // Longitude écliptique

    // Obliquité de l'écliptique (ε)
    const obliq = toRad(23.4393 - 3.563E-7 * T);

    // Ascension Droite (RA)
    const Ra = Math.atan2(Math.cos(obliq) * Math.sin(L), Math.cos(L));

    // Calcul EOT (en radians, puis converti en minutes)
    const delta = (M + w) - Ra;
    const eot = toDeg(delta) * 4; // Conversion en minutes (4 minutes par degré)

    return {
        eot: eot,
        meanAnomaly: toDeg(M),
        eclipticLongitude: toDeg(L)
    };
}

/**
 * Calcule le Temps Sidéral Local Vrai (TSLV) en heures décimales.
 * @param {Date} date - Date UTC
 * @param {number} lon_deg - Longitude en degrés
 * @returns {number} TSLV en heures décimales
 */
function getTrueLocalSiderealTime(date, lon_deg) {
    // JD0 (Jour Julien à 0h UTC)
    const JD0 = Math.floor(getJulianDay(date) - 0.5) + 0.5;
    const T0 = (JD0 - 2451545.0) / 36525;
    // GST (Temps Sidéral de Greenwich)
    let GMST = 6.697374558 + 1.00273790935 * (getJulianDay(date) - JD0) * 24 + 0.06570982441908 * T0 + 1.00273790935 * T0;
    GMST = GMST % 24;

    // TSLV
    let TSLV = GMST + (lon_deg / 15);
    TSLV = TSLV % 24;
    return (TSLV < 0) ? TSLV + 24 : TSLV;
}

// --- UKF SIMULÉ - Fonctions d'état (Pour démonstration) ---
class UKF_21_STATE_FUSION {
    static predict(currentState, lastIMU) {
        // Logique de prédiction simulée
        currentState.status = 'PREDICTION';
        // Simuler l'augmentation de l'incertitude lors de la prédiction sans observation
        currentState.P_v_uncert = Math.min(1000, currentState.P_v_uncert * 1.01); 
        currentState.alt_sigma = Math.min(10, currentState.alt_sigma * 1.005);
        
        // Simuler un mouvement très lent basé sur l'IMU si en PREDICTION
        if (Math.abs(lastIMU.accelX) > 0.5 || Math.abs(lastIMU.accelY) > 0.5) {
            currentState.v_stable += 0.05; // Faux mouvement
        } else {
            currentState.v_stable *= 0.99; // Décrémentation
        }
        return currentState;
    }
    
    static update(currentState, gpsData, imuData) {
        // Logique de mise à jour simulée (UKF complet non implémenté ici)
        const R_mult = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
        currentState.status = 'UPDATE/CORRECTION';
        
        // Mise à jour de la position et réduction de l'incertitude
        currentState.lat = gpsData.lat;
        currentState.lon = gpsData.lon;
        currentState.alt = gpsData.alt;
        currentState.v_stable = gpsData.speed;
        currentState.heading = gpsData.heading;
        
        // Simuler la réduction de l'incertitude après correction
        currentState.P_v_uncert = Math.max(10, gpsData.accuracy * R_mult * 0.5); 
        currentState.alt_sigma = Math.max(1, gpsData.altAccuracy * 0.8 * R_mult);
        currentState.R_v_noise = gpsData.accuracy * R_mult; // Bruit mesuré
        
        return currentState;
    }
}


// Fin du BLOC 1/4 (Encapsulé dans l'IIFE)

((window) => {
    // ... (Début du BLOC 1)
// =================================================================
// BLOC 2/4 : GESTION DES CAPTEURS & UKF
// =================================================================
let lastKnownPosition = null;
let lastKnownPollutants = null;
let lastIMUData = { accelX: 0, accelY: 0, accelZ: EARTH_GRAVITY, magX: NaN, magY: NaN, magZ: NaN, alpha: 0, beta: 0, gamma: 0 };
let gpsWatchId = null;
let lastUpdateTimestamp = Date.now();
let lastGpsTimestamp = 0;
let distanceTotal = 0;
let motionTime = 0;
let totalTime = 0;
let maxSpeed = 0;
let totalSpeed = 0;
let totalSpeedCount = 0;
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let selectedEnvironment = 'NORMAL';
let lastBubblePitch = 0;
let lastBubbleRoll = 0;
let currentSessionStart = Date.now();


// --- FONCTION DE TRAITEMENT IMU (Accéléromètre/Gyro) ---
function processImuData(event) {
    const isAccelerometer = (event.accelerationIncludingGravity || event.rotationRate);
    const isOrientation = (event.alpha !== undefined);

    if (isAccelerometer) {
        // Accéléromètre : Force G (m/s²)
        lastIMUData.accelX = event.accelerationIncludingGravity ? event.accelerationIncludingGravity.x : event.x;
        lastIMUData.accelY = event.accelerationIncludingGravity ? event.accelerationIncludingGravity.y : event.y;
        lastIMUData.accelZ = event.accelerationIncludingGravity ? event.accelerationIncludingGravity.z : event.z;
    }

    if (isOrientation) {
        // Orientation (alpha, beta, gamma)
        lastIMUData.alpha = event.alpha; // Azimut/Cap (Yaw)
        lastIMUData.beta = event.beta; // Inclinaison (Pitch)
        lastIMUData.gamma = event.gamma; // Roulis (Roll)
    }

    if ($('statut-capteur')) {
        $('statut-capteur').textContent = `Actif (${isOrientation ? 'IMU/Or.' : 'IMU'})`;
    }
    
    // Niveau à Bulle (Basé sur Beta/Gamma)
    if ($('bubble')) {
        // Limiter le déplacement à ±45 degrés pour éviter les rotations inutiles
        const pitch = Math.min(45, Math.max(-45, lastIMUData.beta || 0)); 
        const roll = Math.min(45, Math.max(-45, lastIMUData.gamma || 0));

        // Normalisation à la taille du conteneur (100px)
        const bubbleX = (roll / 45) * 45; // Max 45px de déplacement pour 45 degrés
        const bubbleY = (pitch / 45) * 45;

        // Limiter le déplacement pour rester dans le cercle (±45px du centre 50px)
        const limitedX = Math.min(45, Math.max(-45, bubbleX));
        const limitedY = Math.min(45, Math.max(-45, bubbleY));

        $('bubble').style.transform = `translate(${limitedX}px, ${limitedY}px)`;
        lastBubblePitch = pitch;
        lastBubbleRoll = roll;
    }
}

// --- FONCTION DE TRAITEMENT GPS ---
function processGpsData(position) {
    const now = position.timestamp;
    const coords = position.coords;
    const timeDelta = (now - lastGpsTimestamp) / 1000;
    lastGpsTimestamp = now;

    // 1. Initialisation ou mise à jour GPS brute
    if (!lastKnownPosition) {
        lastKnownPosition = { lat: coords.latitude, lon: coords.longitude, alt: coords.altitude || ekfState.alt };
    }

    // Préparation des données pour l'UKF
    const gpsData = {
        lat: coords.latitude,
        lon: coords.longitude,
        alt: coords.altitude || ekfState.alt,
        speed: coords.speed || 0, // speed in m/s
        heading: coords.heading || 0,
        accuracy: coords.accuracy,
        altAccuracy: coords.altitudeAccuracy || 50,
        timeDelta: timeDelta,
    };
    
    // 2. Fusion UKF (Update/Correction)
    ekfState = UKF_21_STATE_FUSION.update(ekfState, gpsData, lastIMUData);

    // 3. Calculs de distance
    if (lastKnownPosition) {
        const from = turf.point([lastKnownPosition.lon, lastKnownPosition.lat]);
        const to = turf.point([gpsData.lon, gpsData.lat]);
        const segmentDistance = turf.distance(from, to, { units: 'meters' });

        // Calcul de la distance 3D (Pythagore)
        const altChange = gpsData.alt - lastKnownPosition.alt;
        const dist3D = Math.sqrt(segmentDistance * segmentDistance + altChange * altChange);

        // Application du rapport de distance (Nether ou autre)
        const ratio = parseFloat($('distance-ratio').textContent);
        distanceTotal += dist3D * ratio;

        // Mise à jour du temps de mouvement
        if (gpsData.speed > 0.1 || dist3D > 1.0) { // Seuil de mouvement
            motionTime += timeDelta;
            totalSpeed += ekfState.v_stable;
            totalSpeedCount++;
            isMoving = true;
        } else {
            isMoving = false;
        }

        // Mise à jour de la vitesse maximale
        const speedKmH = ekfState.v_stable * 3.6;
        if (speedKmH > maxSpeed) {
            maxSpeed = speedKmH;
        }
    }

    // 4. Mise à jour de la position pour le prochain cycle
    lastKnownPosition = { lat: ekfState.lat, lon: ekfState.lon, alt: ekfState.alt };

    // 5. Mise à jour de la carte (implémentée dans le BLOC 4)
    if (typeof updateMap === 'function') {
        updateMap(ekfState.lat, ekfState.lon, ekfState.heading, ekfState.P_v_uncert, distanceTotal);
    }
}

// --- GESTION DES CAPTEURS GPS ---
function toggleGps() {
    const btn = $('toggle-gps-btn');
    if (gpsActive) {
        if (gpsWatchId !== null) {
            navigator.geolocation.clearWatch(gpsWatchId);
            gpsWatchId = null;
        }
        gpsActive = false;
        btn.textContent = '▶️ MARCHE GPS';
        btn.style.backgroundColor = '#28a745';
        ekfState.status = 'PAUSE';
        isMoving = false;
    } else {
        const options = {
            enableHighAccuracy: $('freq-select').value === 'HIGH_FREQ',
            timeout: 5000,
            maximumAge: 0
        };
        gpsWatchId = navigator.geolocation.watchPosition(
            (pos) => { 
                processGpsData(pos);
            },
            (error) => {
                console.error("Erreur GPS:", error);
                $('speed-status-text').textContent = `Erreur GPS: ${error.message}`;
            },
            options
        );
        gpsActive = true;
        btn.textContent = '⏸️ PAUSE GPS';
        btn.style.backgroundColor = '#ffc107'; // Jaune/Orange
        ekfState.status = 'ACQUISITION';
    }
}

// --- SIMULATION MÉTÉO/POLLUANTS (API) ---
async function fetchWeatherAndPollutants(lat, lon) {
    // NOTE: Remplacer par un véritable appel API (e.g., OpenWeatherMap, AQICN) en production.
    // Ici, nous simulerons des données cohérentes avec le contexte (Marseille en Décembre).
    const apiSuccess = Math.random() > 0.1; // 90% de chance de succès

    if (!apiSuccess) {
        lastKnownPollutants = null;
        $('statut-meteo').textContent = 'ÉCHEC API (Défaut)';
        return;
    }

    // Données simulées (basées sur les valeurs entrées + calcul)
    const T_c = 14.4; // Température de l'air fournie par l'utilisateur
    const P_hPa = 1001.3;
    const Humidity_perc = 75.0 + Math.random() * 5; // Simuler une humidité réelle
    
    // Calcul de la densité de l'air (basé sur l'humidité)
    const Pv = 6.112 * Math.exp((17.67 * T_c) / (T_c + 243.5)) * (Humidity_perc / 100);
    const P_dry = P_hPa - Pv;
    const T_K = T_c + 273.15;
    currentAirDensity = (P_dry / (287.058 * T_K)) + (Pv / (461.495 * T_K)); // Équation de l'air humide (approx.)

    // Stockage des dernières valeurs Météo
    lastT_K = T_K;
    lastP_hPa = P_hPa;
    currentSpeedOfSound = getSpeedOfSound(T_K);

    // Stockage des Polluants (simulation)
    lastKnownPollutants = {
        no2: 12 + Math.random() * 5,
        pm25: 5 + Math.random() * 3,
        pm10: 10 + Math.random() * 5,
        o3: 40 + Math.random() * 10,
    };

    $('statut-meteo').textContent = 'ACTIF (API Météo)';
    $('air-temp-c').textContent = `${T_c.toFixed(1)} °C`;
    $('pressure-hpa').textContent = `${P_hPa.toFixed(1)} hPa`;
    $('humidity-perc').textContent = `${Humidity_perc.toFixed(1)} %`;
    $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
    
    // Calcul du point de rosée (simplifié)
    const A = 17.27;
    const B = 237.7;
    const alpha = ((A * T_c) / (B + T_c)) + Math.log(Humidity_perc / 100);
    const dewPoint = (B * alpha) / (A - alpha);
    $('dew-point').textContent = `${dewPoint.toFixed(1)} °C`;
    
    // Mise à jour DOM des polluants (appelée dans la boucle principale aussi)
    updatePollutantsDOM(lastKnownPollutants, false);
}


// =================================================================
// BLOC 3/4 : MISE À JOUR DU DOM (DOM Update Functions)
// =================================================================

function updateTimeDOM() {
    const now = new Date();
    const utcDate = new Date(now.getTime() + now.getTimezoneOffset() * 60000);
    const localTimeFormat = now.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
    const utcDateFormat = utcDate.toLocaleDateString('en-GB', { weekday: 'short', year: 'numeric', month: 'short', day: '2-digit' });
    const utcTimeFormat = utcDate.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });

    $('heure-locale').textContent = localTimeFormat;
    $('date-heure-utc').textContent = `${utcDateFormat} ${utcTimeFormat} GMT`;
    
    // Temps Écoulé
    totalTime = (now.getTime() - currentSessionStart) / 1000;
    $('elapsed-session-time').textContent = `${totalTime.toFixed(2)} s`;
    $('elapsed-motion-time').textContent = `${motionTime.toFixed(2)} s`;
}

function updateAstroDOM() {
    const now = new Date();
    
    // 1. POSITIONNEMENT EKF
    $('lat-ekf').textContent = `${ekfState.lat.toFixed(6)} °`;
    $('lon-ekf').textContent = `${ekfState.lon.toFixed(6)} °`;
    $('alt-ekf').textContent = `${ekfState.alt.toFixed(2)} m`;
    $('heading-display').textContent = ekfState.heading !== null ? `${ekfState.heading.toFixed(1)} °` : 'N/A';
    
    // 2. SOLEIL & LUNE
    const sunPos = SunCalc.getPosition(now, ekfState.lat, ekfState.lon);
    const sunTimes = SunCalc.getTimes(now, ekfState.lat, ekfState.lon);
    const moonPos = SunCalc.getMoonPosition(now, ekfState.lat, ekfState.lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonTimes = SunCalc.getMoonTimes(now, ekfState.lat, ekfState.lon, true);

    // SUN
    $('sun-alt').textContent = `${toDeg(sunPos.altitude).toFixed(1)} °`;
    $('sun-azimuth').textContent = `${toDeg(sunPos.azimuth).toFixed(1)} °`;
    
    const sunrise = sunTimes.sunrise ? formatTime(sunTimes.sunrise) : 'N/A';
    const sunset = sunTimes.sunset ? formatTime(sunTimes.sunset) : 'N/A';
    $('lever-coucher-lune').textContent = moonTimes.rise ? `${formatTime(moonTimes.rise)} / ${moonTimes.set ? formatTime(moonTimes.set) : 'N/A'}` : 'N/A';
    
    const dayDurationMs = sunTimes.sunset - sunTimes.sunrise;
    const dayDurationHours = dayDurationMs / 3600000;
    $('day-duration').textContent = isNaN(dayDurationHours) ? 'N/A' : `${Math.floor(dayDurationHours)}h${Math.round((dayDurationHours % 1) * 60).toString().padStart(2, '0')}min`;
    $('sunrise-times').textContent = sunrise;
    $('coucher-times').textContent = sunset;

    // MOON
    const moonPhaseName = getMoonPhaseName(moonIllum.phase);
    $('moon-phase-name').textContent = moonPhaseName;
    $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
    $('moon-alt').textContent = `${toDeg(moonPos.altitude).toFixed(1)} °`;
    $('moon-azimuth').textContent = `${toDeg(moonPos.azimuth).toFixed(1)} °`;
    $('moon-times').textContent = moonTimes.rise ? `${formatTime(moonTimes.rise)} / ${moonTimes.set ? formatTime(moonTimes.set) : 'N/A'}` : 'N/A';
    
    // 3. TEMPS SOLAIRE & SIDÉRAL (CORRECTION CRITIQUE)
    const jd = getJulianDay(now);
    const eotResult = getEquationOfTime(jd);
    const eotMin = eotResult.eot;
    
    // Heure UTC en heures décimales
    const utcHours = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    
    // Heure Solaire Moyenne (MST) en heures décimales
    const mstHours = utcHours + (ekfState.lon / 15);
    const mstFormatted = formatHours(mstHours);
    
    // Heure Solaire Vraie (TST) en heures décimales
    const tstHours = mstHours + (eotMin / 60);
    const tstFormatted = formatHours(tstHours);
    
    const tslvHours = getTrueLocalSiderealTime(now, ekfState.lon);
    const tslvFormatted = formatHours(tslvHours);

    $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    $('date-solar-mean').textContent = `J=${jd.toFixed(4)}`;
    $('date-solar-true').textContent = `J=${jd.toFixed(4)}`;
    $('tst').textContent = tstFormatted;
    $('mst').textContent = mstFormatted;
    $('noon-solar').textContent = formatHours(12 - (ekfState.lon / 15) - (eotMin / 60)); // Midi solaire vrai (UTC)
    $('eot').textContent = `${eotMin.toFixed(4)} min`;
    $('tslv').textContent = tslvFormatted;
    $('ecl-long').textContent = `${eotResult.eclipticLongitude.toFixed(4)} °`;
    
    updateMinecraftClock(tstHours); // Mise à jour de l'horloge Minecraft

    function formatHours(hours) {
        const h = Math.floor(hours % 24);
        const m = Math.floor((hours * 60) % 60);
        const s = Math.floor((hours * 3600) % 60);
        return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    }
    
    function getMoonPhaseName(phase) {
        if (phase < 0.03 || phase >= 0.97) return 'Nouvelle Lune';
        if (phase < 0.22) return 'Croissant cireux';
        if (phase < 0.28) return 'Premier Quartier';
        if (phase < 0.47) return 'Gibbeuse cireuse';
        if (phase < 0.53) return 'Pleine Lune';
        if (phase < 0.72) return 'Gibbeuse décroissante';
        if (phase < 0.78) return 'Dernier Quartier';
        return 'Croissant décroissant';
    }
} // Fin de updateAstroDOM

function updateSpeedAndRelativityDOM() {
    const v_stable = ekfState.v_stable; // m/s
    const v_kmh = v_stable * 3.6;
    const v_kms = v_stable / 1000;
    const v_raw_ms = lastKnownPosition ? lastKnownPosition.speed : 0; // Vitesse brute GPS

    $('speed-stable').textContent = `${dataOrDefault(v_kmh, 1)} km/h`;
    $('speed-stable-ms').textContent = `${dataOrDefault(v_stable, 2)} m/s`;
    $('speed-stable-kms').textContent = `${dataOrDefault(v_kms, 4, 'e-4')} km/s`;
    $('vitesse-max-session').textContent = `${maxSpeed.toFixed(1)} km/h`;
    $('speed-avg-moving').textContent = totalSpeedCount > 0 ? `${(totalSpeed / totalSpeedCount * 3.6).toFixed(1)} km/h` : '0.0 km/h';
    $('speed-avg-total').textContent = totalTime > 0 ? `${(totalSpeed / (totalTime / (motionTime > 0 ? motionTime : 1)) * 3.6).toFixed(1)} km/h` : '0.0 km/h';

    // Statut
    $('speed-status-text').textContent = gpsActive 
        ? (isMoving ? 'En Mouvement 🚀' : 'Arrêté (GPS Lock)') 
        : 'Pause GPS...';
        
    // Relativité
    const lorentzFactor = 1 / Math.sqrt(1 - (v_stable / C_LIGHT) ** 2);
    const percSpeedC = (v_stable / C_LIGHT) * 100;
    const energy = currentMass * C_LIGHT ** 2 * lorentzFactor;
    const restEnergy = currentMass * C_LIGHT ** 2;
    const momentum = currentMass * v_stable * lorentzFactor;
    const schRad = (2 * G_GRAV * currentMass) / C_LIGHT ** 2;

    $('perc-speed-sound').textContent = `${dataOrDefault((v_stable / currentSpeedOfSound) * 100, 2)} %`;
    $('mach-number').textContent = dataOrDefault(v_stable / currentSpeedOfSound, 4);
    $('percent-speed-light').textContent = `${dataOrDefaultExp(percSpeedC, 2)} %`;
    $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
    $('relativistic-energy').textContent = dataOrDefaultExp(energy, 3, ' J');
    $('rest-mass-energy').textContent = dataOrDefaultExp(restEnergy, 3, ' J');
    $('momentum').textContent = dataOrDefaultExp(momentum, 3, ' kg·m/s');
    $('schwarzschild-radius').textContent = dataOrDefaultExp(schRad, 2, ' m');

    // Mise à jour des constantes météo
    $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Cor.)`;
    $('gravity-base').textContent = `${currentGravity.toFixed(4)} m/s²`;
    
    // Distance
    $('distance-totale').textContent = `${(distanceTotal / 1000).toFixed(3)} km | ${distanceTotal.toFixed(2)} m`;
    
    // Distance lumière
    const distSecLight = distanceTotal / C_LIGHT;
    $('distance-light-s').textContent = dataOrDefaultExp(distSecLight, 2, ' s');
    $('distance-light-min').textContent = dataOrDefaultExp(distSecLight / 60, 2, ' min');
    $('distance-light-h').textContent = dataOrDefaultExp(distSecLight / 3600, 2, ' h');
    $('distance-light-day').textContent = dataOrDefaultExp(distSecLight / 86400, 2, ' j');
    
    // Dilation du temps (Gravité à l'altitude EKF)
    const altitude = ekfState.alt;
    const deltaT_grav = (G_GRAV * currentMass) / (EARTH_RADIUS * C_LIGHT ** 2) * 86400e9; // ns/jour (Approximation)
    $('time-dilation-gravite').textContent = `${deltaT_grav.toFixed(2)} ns/j`;
}

function updateDynamicsDOM() {
    const g_local = 9.8043; // Simulation de la gravité WGS84
    const accX = lastIMUData.accelX;
    const accY = lastIMUData.accelY;
    const accZ = lastIMUData.accelZ;
    const v_stable = ekfState.v_stable; // m/s

    // Mise à jour des données IMU / Accélération
    $('acceleration-x').textContent = `${dataOrDefault(accX, 2)} m/s²`; 
    $('acceleration-z').textContent = `${dataOrDefault(accZ, 2)} m/s²`; 

    $('gravite-wgs84').textContent = `${g_local.toFixed(4)} m/s²`;
    $('force-g-long').textContent = `${dataOrDefault(accX / g_local, 2)} G`;
    $('force-g-vert').textContent = `${dataOrDefault(accZ / g_local, 2)} G`;
    $('vertical-speed').textContent = `${dataOrDefault(ekfState.v_vertical, 2)} m/s`;
    
    // Mécanique des Fluides
    const dynamicPressure = 0.5 * currentAirDensity * v_stable ** 2; // Pa
    const dragCoefficient = 0.47; // Coefficient de traînée (sphère, simulation)
    const crossArea = 0.5; // m² (surface frontale, simulation)
    const dragForce = dynamicPressure * dragCoefficient * crossArea; // N
    const dragPowerKw = (dragForce * v_stable) / 1000; // kW

    $('dynamic-pressure').textContent = `${dataOrDefault(dynamicPressure, 2)} Pa`;
    $('drag-force').textContent = `${dataOrDefault(dragForce, 2)} N`;
    $('drag-power-kw').textContent = `${dataOrDefault(dragPowerKw, 2)} kW`;

    // Champs & Forces
    const kineticEnergy = 0.5 * currentMass * v_stable ** 2;
    const mechanicalPower = currentMass * accX * v_stable; // Puissance mécanique (simplifiée)

    $('kinetic-energy').textContent = `${dataOrDefault(kineticEnergy, 2)} J`;
    $('mechanical-power').textContent = `${dataOrDefault(mechanicalPower, 2)} W`;

    // UKF/EKF Debug
    $('statut-gps-acquisition').textContent = gpsActive ? `Actif (±${dataOrDefault(lastKnownPosition ? lastKnownPosition.accuracy : 0, 1)} m)` : 'Inactif';
    $('statut-ekf-fusion').textContent = ekfState.status;
    $('ukf-v-uncert').textContent = `${dataOrDefault(ekfState.P_v_uncert, 3)} m²/s² (P)`;
    $('ukf-alt-sigma').textContent = `${dataOrDefault(ekfState.alt_sigma, 3)} m (σ)`;
    $('ukf-r-noise').textContent = `${dataOrDefault(ekfState.R_v_noise, 3)} m² (R)`;
    
    // Niveau à Bulle
    $('inclinaison-pitch').textContent = `${dataOrDefault(lastBubblePitch, 1)}°`;
    $('roulis-roll').textContent = `${dataOrDefault(lastBubbleRoll, 1)}°`;
}

function updatePollutantsDOM(data, isDefault) {
    if (!data) {
        $('no2-val').textContent = 'N/A (API Échec)';
        $('pm25-val').textContent = 'N/A (API Échec)';
        $('pm10-val').textContent = 'N/A (API Échec)';
        $('o3-val').textContent = 'N/A (API Échec)';
        return;
    }
    const suffix = isDefault ? ' (Défaut)' : ' (API)';
    $('no2-val').textContent = `${dataOrDefault(data.no2, 2)} µg/m³${suffix}`;
    $('pm25-val').textContent = `${dataOrDefault(data.pm25, 2)} µg/m³${suffix}`;
    $('pm10-val').textContent = `${dataOrDefault(data.pm10, 2)} µg/m³${suffix}`;
    $('o3-val').textContent = `${dataOrDefault(data.o3, 2)} µg/m³${suffix}`;
}

// =================================================================
// BLOC 4/4 : INITIALISATION & GESTION DES ÉVÉNEMENTS
// =================================================================

let map, currentMarker, polyline;

function initMap(lat, lon) {
    map = L.map('map', {
        attributionControl: false, 
        zoomControl: false,
        worldCopyJump: true
    }).setView([lat, lon], 13);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 18,
    }).addTo(map);

    currentMarker = L.circleMarker([lat, lon], {
        color: '#007bff',
        fillColor: '#007bff',
        fillOpacity: 0.9,
        radius: 8
    }).addTo(map);

    polyline = L.polyline([], { color: '#dc3545', weight: 3 }).addTo(map);
}

function updateMap(lat, lon, heading, uncertainty, distance) {
    const latlng = [lat, lon];
    currentMarker.setLatLng(latlng);
    currentMarker.setRadius(Math.max(5, Math.min(20, uncertainty / 10))); // Taille basée sur l'incertitude
    map.panTo(latlng);
    
    // Ajout de la nouvelle position à la polyline
    polyline.addLatLng(latlng);
}

function initDashboard() {
    // Initialisation des données simulées
    if (navigator.geolocation && navigator.geolocation.getCurrentPosition) {
        navigator.geolocation.getCurrentPosition(
            (position) => {
                const lat = position.coords.latitude;
                const lon = position.coords.longitude;
                ekfState.lat = lat;
                ekfState.lon = lon;
                ekfState.alt = position.coords.altitude || 100;
                initMap(lat, lon);
                fetchWeatherAndPollutants(lat, lon);
            },
            (error) => {
                // Utiliser la position de Marseille par défaut si la géolocalisation échoue
                console.warn("Erreur de géolocalisation. Utilisation des coordonnées par défaut.", error);
                initMap(ekfState.lat, ekfState.lon);
                fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
            },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
        );
    } else {
        console.warn("Géolocalisation non supportée. Utilisation des coordonnées par défaut.");
        initMap(ekfState.lat, ekfState.lon);
        fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
    }
    
    // Initialisation de l'IMU (DeviceMotion/DeviceOrientation)
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', processImuData, true);
    }
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', processImuData, true);
    }
    
    // Configuration initiale des sélecteurs
    $('environment-select').value = selectedEnvironment;
    $('mass-input').value = currentMass;
    
    // Gestionnaire d'événements
    $('toggle-gps-btn').addEventListener('click', toggleGps);
    $('emergency-stop-btn').addEventListener('click', () => {
        if (gpsActive) toggleGps();
        alert("ARRÊT D'URGENCE ACTIVÉ: Tous les systèmes GPS/IMU sont coupés.");
        $('emergency-stop-btn').classList.toggle('active');
        $('emergency-stop-btn').textContent = $('emergency-stop-btn').classList.contains('active') 
            ? '🛑 Arrêt d\'urgence: ACTIF 🔴' 
            : '🛑 Arrêt d\'urgence: INACTIF 🟢';
    });
    $('reset-dist-btn').addEventListener('click', () => { distanceTotal = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpeed = 0; });
    $('mass-input').addEventListener('change', (e) => { currentMass = parseFloat(e.target.value) || 0; });
    $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        const env = ENVIRONMENT_FACTORS[selectedEnvironment];
        $('env-factor').textContent = `${env.DISPLAY} (x${env.MULT.toFixed(1)})`;
    });
    
    // Boucle de mise à jour principale (10 Hz)
    setInterval(() => {
        updateTimeDOM();
        
        // Simuler un cycle UKF de prédiction si le GPS est en pause
        if (!gpsActive) {
            ekfState = UKF_21_STATE_FUSION.predict(ekfState, lastIMUData);
        }
        
        updateSpeedAndRelativityDOM();
        updateDynamicsDOM();
        updateAstroDOM();
        
        // Mettre à jour la carte et l'heure Minecraft (simulé)
        if (map) updateMap(ekfState.lat, ekfState.lon, ekfState.heading, ekfState.P_v_uncert, distanceTotal);
        
    }, 100); 

    // Boucle de mise à jour lente (1 fois par minute)
    setInterval(() => {
        if (lastKnownPosition) {
             fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
        }
    }, 60000); 
    
    // Mise à jour de l'heure Minecraft (1 fois par seconde, pour l'icône)
    setInterval(() => {
        const tstHours = formatHoursToDecimal($('tst').textContent);
        updateMinecraftClock(tstHours);
    }, 1000); 

    // Fonctions d'aide pour l'horloge Minecraft
    function updateMinecraftClock(tstHours) {
        const mcClock = $('minecraft-clock');
        const sunEl = $('sun-element');
        const moonEl = $('moon-element');
        const clockStatus = $('clock-status');
        
        // Minecraft: 0h = 6h TST, 12h = 18h TST. Cycle de 24h TST = 360 degrés.
        // Rotation = (TST en heures / 24) * 360
        const rotation = (tstHours / 24) * 360; 
        
        sunEl.style.transform = `rotate(${rotation}deg)`;
        moonEl.style.transform = `rotate(${rotation + 180}deg)`; // Lune est à 180 degrés du Soleil
        
        // Heure Minecraft (MST): 6h = 00:00 (Midi Minecraft), 18h = 12:00 (Minuit Minecraft)
        const mcTimeHours = (tstHours + 6) % 24;
        const mcH = Math.floor(mcTimeHours);
        const mcM = Math.floor((mcTimeHours % 1) * 60);
        $('time-minecraft').textContent = `${mcH.toString().padStart(2, '0')}:${mcM.toString().padStart(2, '0')}`;
        
        // Mise à jour du statut jour/nuit et du fond (Skybox)
        if (mcTimeHours >= 6 && mcTimeHours < 7.5) { // Matin (6h à 7h30)
            clockStatus.textContent = 'Matin/Lever (☀️)';
            mcClock.className = 'sky-sunset';
        } else if (mcTimeHours >= 7.5 && mcTimeHours < 18) { // Jour (7h30 à 18h)
            clockStatus.textContent = 'Jour (☀️)';
            mcClock.className = 'sky-day';
        } else if (mcTimeHours >= 18 && mcTimeHours < 19.5) { // Soir (18h à 19h30)
            clockStatus.textContent = 'Soir/Crépuscule (🌇)';
            mcClock.className = 'sky-sunset';
        } else { // Nuit (19h30 à 6h)
            clockStatus.textContent = 'Nuit (🌙)';
            mcClock.className = 'sky-night';
        }
        
    }
    
    function formatHoursToDecimal(timeString) {
        if (!timeString || timeString === 'N/A') return 0;
        const parts = timeString.split(':').map(Number);
        return parts[0] + parts[1] / 60 + parts[2] / 3600;
    }

}

window.onload = initDashboard;

})(window);

// --- FIN DU FICHIER JAVASCRIPT COMPLET ---
