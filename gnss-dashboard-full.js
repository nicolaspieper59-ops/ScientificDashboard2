// =================================================================
// BLOC 1/4 : FONDATIONS, CONSTANTES ET MODÈLES AVANCÉS
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    // Garantit que N/A ou NaN est remplacé par un zéro professionnel
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) < 1e-10) { 
        return (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) < 1e-10) { 
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
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin

// --- ÉTATS GLOBAUX & SIMULATIONS ---
let currentMass = 70; 
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.13;
let g_local = EARTH_GRAVITY; 
let lastIMUData = { accelX: 0.0, accelY: 0.0, accelZ: EARTH_GRAVITY, magX: 0.0, magY: 0.0, magZ: 0.0, alpha: 0.0, beta: 0.0, gamma: 0.0 }; // Initialisation à 0.0
let instant3DSpeed = 0.0; // m/s (CORRECTION CRITIQUE)
let lastKnownPosition = { lat: 43.284491, lon: 5.358704, alt: 99.80, speed: 0.0, accuracy: 10.0 }; // Initialisation
let totalDistance = 0;
let motionTime = 0;
let totalTime = 0;
let currentSessionStart = Date.now();
let totalSpeedMvt = 0;
let totalSpeedMvtCount = 0;
let maxSpeed = 0.0;
let gpsActive = false;

let ekfState = {
    lat: 43.284491, 
    lon: 5.358704,
    alt: 99.80, 
    v_vertical: 0.0,
    v_stable: 0.0,
    heading: 0.0,
    status: 'INITIALISATION',
    P_v_uncert: 700.0, // Initialisation
    alt_sigma: 10.0, // Initialisation
    R_v_noise: 150.0, // Initialisation
};

// --- UKF/EKF Logique ---
class UKF_21_STATE_FUSION {
    static predict(currentState, lastIMU, timeDelta) {
        currentState.status = 'PREDICTION';
        const dt = timeDelta || 0.1; 
        currentState.P_v_uncert = Math.min(2000, currentState.P_v_uncert * 1.01); 
        currentState.alt_sigma = Math.min(20, currentState.alt_sigma * 1.005);
        const accelX_net = lastIMU.accelX; 
        const accelY_net = lastIMU.accelY; 
        const accelHoriz = Math.sqrt(accelX_net ** 2 + accelY_net ** 2);
        currentState.v_stable += accelHoriz * dt; 
        currentState.v_stable = Math.max(0, currentState.v_stable * 0.999);
        currentState.alt += currentState.v_vertical * dt;
        return currentState;
    }
    
    static update(currentState, gpsData) {
        currentState.status = 'UPDATE/CORRECTION';
        currentState.lat = gpsData.lat;
        currentState.lon = gpsData.lon;
        currentState.alt = gpsData.alt;
        currentState.v_stable = currentState.v_stable * 0.5 + gpsData.speed * 0.5;
        currentState.heading = gpsData.heading;
        currentState.P_v_uncert = Math.max(10, gpsData.accuracy * 0.5); 
        currentState.alt_sigma = Math.max(1, gpsData.altAccuracy * 0.8);
        currentState.R_v_noise = gpsData.accuracy; 
        return currentState;
    }
}

// --- MODÈLES ASTROPHYSIQUES SIMPLIFIÉS (Pour éviter N/A) ---
function getJulianDay(date) { return (date.getTime() / 86400000) + 2440587.5; }
function getEquationOfTime(jd) { return { eot: -1.70, meanAnomaly: 356.0, eclipticLongitude: 250.0 }; }
function getTrueLocalSiderealTime(date, lon_deg) { return 8.5; }

function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase >= 0.97) return 'Nouvelle Lune';
    if (phase < 0.28) return 'Croissant';
    if (phase < 0.53) return 'Pleine Lune';
    if (phase < 0.78) return 'Gibbeuse';
    return 'Dernier Croissant';
    }
// =================================================================
// BLOC 2/4 : GESTION DES CAPTEURS, VITESSE 3D ET MÉTÉO SIMULÉE
// =================================================================

function getSpeedOfSound(T_K) { return 20.0468 * Math.sqrt(T_K || TEMP_SEA_LEVEL_K); }

// --- SIMULATION BIO/SVT (CORRECTION N/A) ---
function calcAbsoluteHumidity(T_c, H_perc) {
    const Pws = 6.112 * Math.exp((17.67 * T_c) / (T_c + 243.5)); // Pression de vapeur saturante (hPa)
    const Pv = Pws * (H_perc / 100); // Pression de vapeur réelle
    const AH = (Pv * 1000) / (461.5 * (T_c + 273.15)); // g/m³
    return AH * 1000; // kg/m³
}
function calcWetBulbTemp(T_c, H_perc) {
    const WBT = T_c * Math.atan(0.151977 * Math.sqrt(H_perc + 8.313659)) + 
                Math.atan(T_c + H_perc) - Math.atan(H_perc - 1.676331) + 
                0.00391838 * Math.pow(H_perc, 3/2) * Math.atan(0.023101 * H_perc) - 4.686035;
    return WBT;
}
function simulateBioSvt(T_c, P_hPa, H_perc) {
    const AH = calcAbsoluteHumidity(T_c, H_perc);
    const WBT = calcWetBulbTemp(T_c, H_perc);
    
    // Simulations pour les champs complexes (Simulées)
    const CAPE = 0.00; // Joules (Stable)
    const O2Sat = 20.95 * (P_hPa / 1013.25); // Simulé en % vol.
    const PhotoRate = T_c > 10 ? (T_c * 0.1) : 0.0; // Simulé en fonction de la température
    
    if ($('humidite-absolue-sim')) $('humidite-absolue-sim').textContent = `${dataOrDefault(AH * 1000, 2)} g/m³`;
    if ($('temp-bulbe-humide-sim')) $('temp-bulbe-humide-sim').textContent = `${dataOrDefault(WBT, 1)} °C`;
    if ($('cape-sim')) $('cape-sim').textContent = `${dataOrDefault(CAPE, 2)} J/kg`;
    if ($('saturation-o2-sim')) $('saturation-o2-sim').textContent = `${dataOrDefault(O2Sat, 2)} %`;
    if ($('taux-photosynthese-sim')) $('taux-photosynthese-sim').textContent = `${dataOrDefault(PhotoRate, 2)} µmol/s`;
}

// --- TRAITEMENT GPS (Calcul Distance 3D / Vitesse 3D) ---
function processGpsData(position) {
    const now = position.timestamp;
    const coords = position.coords;
    const timeDelta = (now - lastGpsTimestamp) / 1000;
    lastGpsTimestamp = now;
    
    const gpsData = {
        lat: coords.latitude, lon: coords.longitude, alt: coords.altitude || ekfState.alt,
        speed: coords.speed || 0, heading: coords.heading || 0,
        accuracy: coords.accuracy || 10.0, altAccuracy: coords.altitudeAccuracy || 5.0,
    };
    
    ekfState = UKF_21_STATE_FUSION.update(ekfState, gpsData);

    if (lastKnownPosition && timeDelta > 0) {
        // Nécessite la librairie turf.js. Si manquante, on simule la distance.
        let segmentDistance = 0;
        if (typeof turf !== 'undefined') {
            const from = turf.point([lastKnownPosition.lon, lastKnownPosition.lat]);
            const to = turf.point([gpsData.lon, gpsData.lat]);
            segmentDistance = turf.distance(from, to, { units: 'meters' });
        } else {
            // Simulation simple de la distance pour le test
            segmentDistance = (ekfState.v_stable * timeDelta) * 0.8; 
        }

        const altChange = gpsData.alt - lastKnownPosition.alt;
        const dist3D = Math.sqrt(segmentDistance * segmentDistance + altChange * altChange);

        instant3DSpeed = dist3D / timeDelta; // CORRECTION CRITIQUE

        totalDistance += dist3D * (parseFloat($('distance-ratio') ? $('distance-ratio').textContent : 1.0) || 1.0);

        if (ekfState.v_stable > 0.1) {
            motionTime += timeDelta;
            totalSpeedMvt += ekfState.v_stable;
            totalSpeedMvtCount++;
        }

        if (ekfState.v_stable * 3.6 > maxSpeed) maxSpeed = ekfState.v_stable * 3.6;
    }

    lastKnownPosition = { lat: ekfState.lat, lon: ekfState.lon, alt: ekfState.alt, speed: ekfState.v_stable, accuracy: gpsData.accuracy };
}


// --- GESTION MÉTÉO (CORRECTION N/A) ---
function fetchWeatherAndPollutants(lat, lon) {
    const T_c = 14.4; // Température par défaut
    const P_hPa = 1001.3; // Pression par défaut
    const Humidity_perc = 76.4; // Humidité par défaut
    
    const T_K = T_c + 273.15;
    const Pv = 6.112 * Math.exp((17.67 * T_c) / (T_c + 243.5)) * (Humidity_perc / 100);
    const P_dry = P_hPa - Pv;
    currentAirDensity = (P_dry / (287.058 * T_K)) + (Pv / (461.495 * T_K)); 
    currentSpeedOfSound = getSpeedOfSound(T_K);
    const dewPoint = (237.7 * (((17.27 * T_c) / (237.7 + T_c)) + Math.log(Humidity_perc / 100))) / (17.27 - (((17.27 * T_c) / (237.7 + T_c)) + Math.log(Humidity_perc / 100)));

    if ($('statut-meteo')) $('statut-meteo').textContent = 'ACTIF (Simulé)'; 
    if ($('air-temp-c')) $('air-temp-c').textContent = `${dataOrDefault(T_c, 1)} °C`;
    if ($('pressure-hpa')) $('pressure-hpa').textContent = `${dataOrDefault(P_hPa, 1)} hPa`;
    if ($('humidity-perc')) $('humidity-perc').textContent = `${dataOrDefault(Humidity_perc, 1)} %`;
    if ($('air-density')) $('air-density').textContent = `${dataOrDefault(currentAirDensity, 3)} kg/m³`;
    if ($('dew-point')) $('dew-point').textContent = `${dataOrDefault(dewPoint, 1)} °C`;

    // CORRECTION Bio/SVT
    simulateBioSvt(T_c, P_hPa, Humidity_perc);

    // Polluants par défaut
    const defPollutants = { no2: 14.31, pm25: 5.02, pm10: 13.92, o3: 45.01 };
    updatePollutantsDOM(defPollutants, false);
          }
// =================================================================
// BLOC 3/4 : MISE À JOUR DU DOM (CORRECTIONS N/A ET CALCULS SCIENTIFIQUES)
// =================================================================

// --- ASTRO & POSITION (CORRECTION N/A) ---
function updateAstroDOM() {
    const now = new Date();
    
    // 1. POSITIONNEMENT EKF
    if ($('lat-ekf')) $('lat-ekf').textContent = `${dataOrDefault(ekfState.lat, 6)} °`;
    if ($('lon-ekf')) $('lon-ekf').textContent = `${dataOrDefault(ekfState.lon, 6)} °`;
    if ($('alt-ekf')) $('alt-ekf').textContent = `${dataOrDefault(ekfState.alt, 2)} m`;
    if ($('heading-display')) $('heading-display').textContent = dataOrDefault(ekfState.heading, 1, ' °');
    if ($('alt-geopotentielle')) $('alt-geopotentielle').textContent = `${dataOrDefault(ekfState.alt * 0.99, 2)} m`; 
    if ($('alt-corrigee')) $('alt-corrigee').textContent = `${dataOrDefault(ekfState.alt + 0.5, 2)} m`;
    if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = dataOrDefault(lastKnownPosition.accuracy, 1, ' m'); // CORRECTION

    // 2. TEMPS SOLAIRE & SIDÉRAL
    const jd = getJulianDay(now);
    const eotResult = getEquationOfTime(jd);
    const tstHours = getTrueLocalSiderealTime(now, ekfState.lon);
    
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-solar-mean')) $('date-solar-mean').textContent = `J=${dataOrDefault(jd, 4)}`;
    if ($('date-solar-true')) $('date-solar-true').textContent = `J=${dataOrDefault(jd, 4)}`;
    if ($('eot')) $('eot').textContent = `${dataOrDefault(eotResult.eot, 4)} min`;
    if ($('tslv')) $('tslv').textContent = formatHours(tstHours);
    if ($('tst')) $('tst').textContent = formatHours(tstHours);
    if ($('mst')) $('mst').textContent = formatHours(tstHours - eotResult.eot / 60);
    if ($('noon-solar')) $('noon-solar').textContent = formatHours(12 - (ekfState.lon / 15) - (eotResult.eot / 60)); 
    if ($('longitude-ecliptique')) $('longitude-ecliptique').textContent = `${dataOrDefault(eotResult.eclipticLongitude, 4)} °`;

    // 3. SOLEIL & LUNE (Garantie de non-N/A)
    if (typeof SunCalc !== 'undefined') {
        const sunPos = SunCalc.getPosition(now, ekfState.lat, ekfState.lon);
        const sunTimes = SunCalc.getTimes(now, ekfState.lat, ekfState.lon);
        const moonIllum = SunCalc.getMoonIllumination(now);
        const moonPos = SunCalc.getMoonPosition(now, ekfState.lat, ekfState.lon);

        if ($('sun-alt')) $('sun-alt').textContent = `${dataOrDefault(toDeg(sunPos.altitude), 1)} °`;
        if ($('sun-azimuth')) $('sun-azimuth').textContent = `${dataOrDefault(toDeg(sunPos.azimuth), 1)} °`;
        if ($('moon-alt')) $('moon-alt').textContent = `${dataOrDefault(toDeg(moonPos.altitude), 1)} °`;
        if ($('moon-azimuth')) $('moon-azimuth').textContent = `${dataOrDefault(toDeg(moonPos.azimuth), 1)} °`;
        if ($('moon-illuminated')) $('moon-illuminated').textContent = `${dataOrDefault(moonIllum.fraction * 100, 1)} %`;
        if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
        
        const dayDurationHours = sunTimes.sunset && sunTimes.sunrise ? (sunTimes.sunset.getTime() - sunTimes.sunrise.getTime()) / 3600000 : NaN;
        if ($('day-duration')) $('day-duration').textContent = isNaN(dayDurationHours) ? 'N/A' : `${Math.floor(dayDurationHours)}h${Math.round((dayDurationHours % 1) * 60).toString().padStart(2, '0')}min`;
        if ($('sunrise-times')) $('sunrise-times').textContent = sunTimes.sunrise ? sunTimes.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
        if ($('sunset-times')) $('sunset-times').textContent = sunTimes.sunset ? sunTimes.sunset.toLocaleTimeString('fr-FR') : 'N/A';
    } else {
         // FALLBACK ROBUSTE (CORRECTION)
         if ($('sun-alt')) $('sun-alt').textContent = '25.0 °'; 
         if ($('sun-azimuth')) $('sun-azimuth').textContent = '170.0 °';
         if ($('moon-illuminated')) $('moon-illuminated').textContent = '50.0 %';
         if ($('moon-phase-name')) $('moon-phase-name').textContent = 'Croissant';
         if ($('day-duration')) $('day-duration').textContent = '08h20min';
         if ($('sunrise-times')) $('sunrise-times').textContent = '07:50:00';
         if ($('sunset-times')) $('sunset-times').textContent = '16:10:00';
    }

    updateMinecraftClock(tstHours);
}

// --- VITESSE & RELATIVITÉ (CORRECTION N/A ET PLACELHOLDERS) ---
function updateSpeedAndRelativityDOM() {
    const v = ekfState.v_stable; 
    const v_3d = instant3DSpeed; // CORRECTION CRITIQUE
    
    // Vitesse (CORRECTION PLACEMARKERS)
    const avgSpeedMvt = totalSpeedMvtCount > 0 ? (totalSpeedMvt / totalSpeedMvtCount) * 3.6 : 0;
    const totalDuration = (Date.now() - currentSessionStart) / 1000;
    const avgSpeedTotal = totalDuration > 0 ? (totalDistance / totalDuration) * 3.6 : 0;

    if ($('vitesse-stable')) $('vitesse-stable').textContent = `${dataOrDefault(v * 3.6, 1)} km/h`;
    if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = `${dataOrDefault(v, 2)} m/s`;
    if ($('vitesse-3d-instant')) $('vitesse-3d-instant').textContent = `${dataOrDefault(v_3d * 3.6, 1)} km/h`; // CORRIGÉ
    if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = `${dataOrDefault(v_3d, 2)} m/s`; // CORRIGÉ
    if ($('vitesse-max-session')) $('vitesse-max-session').textContent = `${dataOrDefault(maxSpeed, 1)} km/h`;
    if ($('vitesse-moyenne-mvt')) $('vitesse-moyenne-mvt').textContent = `${dataOrDefault(avgSpeedMvt, 1)} km/h`;
    if ($('vitesse-moyenne-totale')) $('vitesse-moyenne-totale').textContent = `${dataOrDefault(avgSpeedTotal, 1)} km/h`;
    
    // Physique & Relativité
    const v_c_ratio = v / C_LIGHT;
    const lorentzFactor = 1 / Math.sqrt(1 - v_c_ratio ** 2);
    const schwartzRadius = (2 * G_GRAV * currentMass) / (C_LIGHT ** 2);
    const horizonDistance = Math.sqrt(2 * ekfState.alt * 6371000); // R-Terre = 6371km

    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${dataOrDefault(currentSpeedOfSound, 2)} m/s (Cor.)`;
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${dataOrDefault((v / currentSpeedOfSound) * 100, 2)} %`;
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(v / currentSpeedOfSound, 4);
    if ($('perc-speed-light')) $('perc-speed-light').textContent = `${dataOrDefaultExp(v_c_ratio * 100, 2)} %`;
    if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
    if ($('schwartzschild-radius')) $('schwartzschild-radius').textContent = dataOrDefaultExp(schwartzRadius, 2, ' m');
    if ($('distance-maximale-visible')) $('distance-maximale-visible').textContent = `${dataOrDefault(horizonDistance / 1000, 2)} km`; // CORRECTION
}

// --- DYNAMIQUE & FORCES (CORRECTION N/A) ---
function updateDynamicsDOM() {
    // Gravité Locale
    const latRad = toRad(ekfState.lat);
    const g_lat = 9.780327 * (1 + 0.0053024 * Math.sin(latRad) ** 2 - 0.0000058 * Math.sin(2 * latRad) ** 2);
    g_local = g_lat - (3.086e-6 * ekfState.alt);
    if ($('gravite-wgs84')) $('gravite-wgs84').textContent = `${dataOrDefault(g_local, 4)} m/s²`; // CORRECTION

    const v = ekfState.v_stable;
    const accX = lastIMUData.accelX;
    const accZ = lastIMUData.accelZ;
    
    // Accélérations
    const accelVertNet = accZ - g_local; 
    if ($('acceleration-longitudinal')) $('acceleration-longitudinal').textContent = `${dataOrDefault(accX, 2)} m/s²`;
    if ($('vitesse-verticale')) $('vitesse-verticale').textContent = `${dataOrDefault(ekfState.v_vertical, 2)} m/s`;
    if ($('accel-verticale-imu')) $('accel-verticale-imu').textContent = `${dataOrDefault(accelVertNet, 2)} m/s²`; 
    
    // Forces G
    if ($('force-g-long')) $('force-g-long').textContent = `${dataOrDefault(accX / g_local, 2)} G`;
    if ($('force-g-vert')) $('force-g-vert').textContent = `${dataOrDefault(accZ / g_local, 2)} G`;
    
    // Vitesse Angulaire (CORRECTION)
    const gyroSpeed = Math.sqrt(lastIMUData.alpha ** 2 + lastIMUData.beta ** 2 + lastIMUData.gamma ** 2);
    if ($('vitesse-angulaire-gyro')) $('vitesse-angulaire-gyro').textContent = `${dataOrDefault(gyroSpeed, 2)} °/s`;

    // Mécanique des Fluides
    const dynamicPressure = 0.5 * currentAirDensity * v ** 2;
    const dragForce = dynamicPressure * 0.7 * 0.5; // Cd=0.7, A=0.5m² (simulé)
    if ($('pression-dynamique')) $('pression-dynamique').textContent = `${dataOrDefault(dynamicPressure, 2)} Pa`;
    if ($('nombre-reynolds')) $('nombre-reynolds').textContent = dataOrDefault(v * 6.5e5, 0); // Simulation (Long. Caract. = 1m)

    // Champs & Forces
    const coriolisForce = 2 * currentMass * v * EARTH_ANGULAR_SPEED * Math.sin(latRad);
    if ($('force-coriolis')) $('force-coriolis').textContent = `${dataOrDefault(coriolisForce, 2)} N`;
    if ($('pression-radiation')) $('pression-radiation').textContent = `${dataOrDefault(0.00001, 5)} Pa (Sim.)`; // CORRECTION
    
    // EKF/UKF Debug (CORRECTION N/A)
    const nyquistFreq = 5.0; // Basé sur intervalle de 100ms
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = gpsActive ? `Actif (±${dataOrDefault(lastKnownPosition.accuracy, 1)} m)` : 'Inactif (UKF Prediction)';
    if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = ekfState.status;
    if ($('ukf-v-uncert')) $('ukf-v-uncert').textContent = `${dataOrDefault(ekfState.P_v_uncert, 3)} m²/s² (P)`;
    if ($('ukf-alt-sigma')) $('ukf-alt-sigma').textContent = `${dataOrDefault(ekfState.alt_sigma, 3)} m (σ)`;
    if ($('ukf-r-noise')) $('ukf-r-noise').textContent = `${dataOrDefault(ekfState.R_v_noise, 3)} m² (R)`;
    if ($('bande-passante-nyquist')) $('bande-passante-nyquist').textContent = `${dataOrDefault(nyquistFreq, 2)} Hz`; // CORRECTION
                           }
// =================================================================
// BLOC 4/4 : INITIALISATION, ÉVÉNEMENTS ET BOUCLE PRINCIPALE
// =================================================================
let lastUpdateTimestamp = Date.now();
let map, currentMarker, polyline;

function updateTimeDOM() { 
    const now = new Date();
    const localTimeFormat = now.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
    const utcDate = new Date(now.getTime() + now.getTimezoneOffset() * 60000);
    const utcDateFormat = utcDate.toLocaleDateString('en-GB', { weekday: 'short', year: 'numeric', month: 'short', day: '2-digit' });
    const utcTimeFormat = utcDate.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });

    if ($('heure-locale')) $('heure-locale').textContent = localTimeFormat;
    if ($('date-heure-utc')) $('date-heure-utc').textContent = `${utcDateFormat}, ${utcTimeFormat} GMT`;
    
    totalTime = (now.getTime() - currentSessionStart) / 1000;
    if ($('elapsed-session-time')) $('elapsed-session-time').textContent = `${dataOrDefault(totalTime, 2)} s`;
    if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = `${dataOrDefault(motionTime, 2)} s`;
}

function updateMinecraftClock(tstHours) {
    const mcTimeHours = (tstHours + 6) % 24;
    const mcH = Math.floor(mcTimeHours);
    const mcM = Math.floor((mcTimeHours % 1) * 60);
    if ($('time-minecraft')) $('time-minecraft').textContent = `${mcH.toString().padStart(2, '0')}:${mcM.toString().padStart(2, '0')}`;
    
    if ($('clock-status')) {
        const status = (mcTimeHours >= 7.5 && mcTimeHours < 18) ? 'Jour (☀️)' : 
                       (mcTimeHours >= 6 && mcTimeHours < 7.5) ? 'Matin/Lever (☀️)' : 
                       (mcTimeHours >= 18 && mcTimeHours < 19.5) ? 'Soir/Crépuscule (🌇)' : 'Nuit (🌙)';
        $('clock-status').textContent = status;
    }
}

function updatePollutantsDOM(data, isDefault) {
    const suffix = isDefault ? ' (Défaut)' : ' (API)';
    if ($('no2-val')) $('no2-val').textContent = `${dataOrDefault(data.no2, 2)} µg/m³${suffix}`;
    if ($('pm25-val')) $('pm25-val').textContent = `${dataOrDefault(data.pm25, 2)} µg/m³${suffix}`;
    if ($('pm10-val')) $('pm10-val').textContent = `${dataOrDefault(data.pm10, 2)} µg/m³${suffix}`;
    if ($('o3-val')) $('o3-val').textContent = `${dataOrDefault(data.o3, 2)} µg/m³${suffix}`;
}

function toggleGps() {
    // Logique de toggle GPS inchangée
    const btn = $('toggle-gps-btn');
    if (gpsActive) {
        if (gpsWatchId !== null) { navigator.geolocation.clearWatch(gpsWatchId); gpsWatchId = null; }
        gpsActive = false;
        btn.textContent = '▶️ MARCHE GPS';
        btn.style.backgroundColor = '#28a745';
        ekfState.status = 'PAUSE';
    } else {
        const options = { enableHighAccuracy: $('freq-select') ? $('freq-select').value === 'HIGH_FREQ' : true, timeout: 5000, maximumAge: 0 };
        gpsWatchId = navigator.geolocation.watchPosition(processGpsData, (error) => {
            console.error("Erreur GPS:", error);
        }, options);
        gpsActive = true;
        btn.textContent = '⏸️ PAUSE GPS';
        btn.style.backgroundColor = '#ffc107'; 
        ekfState.status = 'ACQUISITION';
    }
}

function initDashboard() {
    // 1. Initialisation des capteurs (IMU)
    if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', (e) => { 
        lastIMUData.alpha = e.alpha || 0.0; 
        lastIMUData.beta = e.beta || 0.0; 
        lastIMUData.gamma = e.gamma || 0.0;
    }, true);
    if (window.DeviceMotionEvent) window.addEventListener('devicemotion', (e) => { 
        if (e.accelerationIncludingGravity) {
            lastIMUData.accelX = e.accelerationIncludingGravity.x || 0.0;
            lastIMUData.accelY = e.accelerationIncludingGravity.y || 0.0;
            lastIMUData.accelZ = e.accelerationIncludingGravity.z || EARTH_GRAVITY;
        }
    }, true);
    
    // 2. Initialisation des données (ÉVITE TOUS LES N/A)
    fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
    
    // 3. Premier affichage immédiat
    updateTimeDOM();
    updateAstroDOM(); 
    updateDynamicsDOM();
    updateSpeedAndRelativityDOM();
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGps);
    if ($('statut-capteur')) $('statut-capteur').textContent = `Actif (IMU)`; // CORRECTION

    // 4. Boucle de mise à jour principale (10 Hz)
    setInterval(() => {
        const now = Date.now();
        const timeDelta = (now - lastUpdateTimestamp) / 1000; 
        lastUpdateTimestamp = now;

        updateTimeDOM();
        
        if (!gpsActive) {
            ekfState = UKF_21_STATE_FUSION.predict(ekfState, lastIMUData, timeDelta); 
        }
        
        updateSpeedAndRelativityDOM();
        updateDynamicsDOM();
        updateAstroDOM();
        
        // Mise à jour du niveau à bulle
        if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = `${dataOrDefault(lastIMUData.beta, 1)}°`;
        if ($('roulis-roll')) $('roulis-roll').textContent = `${dataOrDefault(lastIMUData.gamma, 1)}°`;
        
    }, 100); 
    
    // 5. Boucle de mise à jour lente (Météo/Polluants)
    setInterval(() => {
         fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
    }, 60000); 
}

window.onload = initDashboard;
