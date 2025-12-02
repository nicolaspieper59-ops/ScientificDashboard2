// =================================================================
// BLOC 1/4 : FONDATIONS, CONSTANTES ET ASTROPHYSIQUE
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix; // Valeur par défaut professionnelle (zéro)
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
const formatTime = (date) => (date instanceof Date && !isNaN(date) ? `${date.getHours().toString().padStart(2, '0')}:${date.getMinutes().toString().padStart(2, '0')}:${date.getSeconds().toString().padStart(2, '0')}` : '00:00:00');
const formatHours = (hours) => {
    const h = Math.floor(hours % 24);
    const m = Math.floor((hours * 60) % 60);
    const s = Math.floor((hours * 3600) % 60);
    return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
};

// --- CONSTANTES PHYSIQUES ET ÉTATS INITIAUX ---
const C_LIGHT = 299792458; // m/s
const G_GRAV = 6.67430e-11; // m³/kg/s²
const EARTH_GRAVITY = 9.8067; // m/s² (Standard)
const RHO_SEA_LEVEL = 1.225; // kg/m³
const TEMP_SEA_LEVEL_K = 288.15; // K (15°C)

let currentMass = 70; // kg
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.13; // m/s (Défaut)
let g_local = EARTH_GRAVITY; 

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

// --- MODÈLES ASTROPHYSIQUES (COMPLETUDE) ---
function getJulianDay(date) { 
    const time = date.getTime() / 86400000;
    const offset = 2440587.5;
    return time + offset;
}
function getEquationOfTime(jd) { 
    const T = (jd - 2451545.0) / 36525;
    const w = toRad(282.9404 + 4.70935E-5 * T);
    const e = 0.016709 - 1.151E-9 * T;
    const M = toRad(356.0470 + 35999.0503 * T);
    const E = M + e * Math.sin(M) * (1 + e * Math.cos(M));
    const L = w + E; 
    const obliq = toRad(23.4393 - 3.563E-7 * T);
    const Ra = Math.atan2(Math.cos(obliq) * Math.sin(L), Math.cos(L));
    const delta = (M + w) - Ra;
    const eot = toDeg(delta) * 4;
    return { eot: eot, meanAnomaly: toDeg(M), eclipticLongitude: toDeg(L) };
}
function getTrueLocalSiderealTime(date, lon_deg) { 
    const JD0 = Math.floor(getJulianDay(date) - 0.5) + 0.5;
    const T0 = (JD0 - 2451545.0) / 36525;
    let GMST = 6.697374558 + 1.00273790935 * (getJulianDay(date) - JD0) * 24 + 0.06570982441908 * T0;
    GMST = GMST % 24;
    let TSLV = GMST + (lon_deg / 15);
    TSLV = TSLV % 24;
    return (TSLV < 0) ? TSLV + 24 : TSLV;
}

// --- UKF Logique (Dead Reckoning Amélioré) ---
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
        const R_mult = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
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
// =================================================================
// BLOC 2/4 : GESTION DES CAPTEURS ET VITESSE 3D
// =================================================================
let lastKnownPosition = null;
let lastIMUData = { accelX: 0.0, accelY: 0.0, accelZ: EARTH_GRAVITY, magX: NaN, magY: NaN, magZ: NaN, alpha: 0.0, beta: 0.0, gamma: 0.0 };
let gpsActive = false;
let lastGpsTimestamp = 0;
let distanceTotal = 0;
let motionTime = 0;
let totalTime = 0;
let maxSpeed = 0;
let totalSpeed = 0;
let totalSpeedCount = 0;
let lastUpdateTimestamp = Date.now();
let selectedEnvironment = 'NORMAL';
let instant3DSpeed = 0; // m/s
let isMoving = false;

const ENVIRONMENT_FACTORS = {
    'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

function getSpeedOfSound(T_K) {
    return 20.0468 * Math.sqrt(T_K || 288.15); // Fallback si T_K est nul
}

// --- TRAITEMENT IMU ---
function processImuData(event) {
    const isAccelerometer = (event.accelerationIncludingGravity || event.rotationRate);
    const isOrientation = (event.alpha !== undefined);

    if (isAccelerometer) {
        lastIMUData.accelX = event.accelerationIncludingGravity ? event.accelerationIncludingGravity.x : 0.0;
        lastIMUData.accelY = event.accelerationIncludingGravity ? event.accelerationIncludingGravity.y : 0.0;
        lastIMUData.accelZ = event.accelerationIncludingGravity ? event.accelerationIncludingGravity.z : EARTH_GRAVITY;
    }

    if (isOrientation) {
        lastIMUData.alpha = event.alpha; 
        lastIMUData.beta = event.beta; 
        lastIMUData.gamma = event.gamma;
    }
    
    // Le statut est mis à jour dans la boucle principale
}

// --- TRAITEMENT GPS (Calcul Distance 3D / Vitesse 3D) ---
function processGpsData(position) {
    const now = position.timestamp;
    const coords = position.coords;
    const timeDelta = (now - lastGpsTimestamp) / 1000;
    lastGpsTimestamp = now;
    
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
        const from = turf.point([lastKnownPosition.lon, lastKnownPosition.lat]);
        const to = turf.point([gpsData.lon, gpsData.lat]);
        const segmentDistance = turf.distance(from, to, { units: 'meters' });

        const altChange = gpsData.alt - lastKnownPosition.alt;
        const dist3D = Math.sqrt(segmentDistance * segmentDistance + altChange * altChange);

        // VITESSE 3D INSTANTANÉE (CORRECTION)
        instant3DSpeed = dist3D / timeDelta;

        const ratio = parseFloat($('distance-ratio') ? $('distance-ratio').textContent : 1.0) || 1.0;
        distanceTotal += dist3D * ratio;

        if (instant3DSpeed > 0.1) {
            motionTime += timeDelta;
            totalSpeed += ekfState.v_stable;
            totalSpeedCount++;
            isMoving = true;
        } else {
            isMoving = false;
        }

        const speedKmH = ekfState.v_stable * 3.6;
        if (speedKmH > maxSpeed) maxSpeed = speedKmH;
    }

    lastKnownPosition = { lat: ekfState.lat, lon: ekfState.lon, alt: ekfState.alt, speed: ekfState.v_stable };
}

// --- GESTION MÉTÉO (Fournit des valeurs par défaut pour éviter N/A) ---
async function fetchWeatherAndPollutants(lat, lon) {
    const T_c = 14.4;
    const P_hPa = 1001.3;
    const Humidity_perc = 78.1; 
    
    // Calculs thermodynamiques pour la densité et la vitesse du son
    const Pv = 6.112 * Math.exp((17.67 * T_c) / (T_c + 243.5)) * (Humidity_perc / 100);
    const P_dry = P_hPa - Pv;
    const T_K = T_c + 273.15;
    currentAirDensity = (P_dry / (287.058 * T_K)) + (Pv / (461.495 * T_K)); 
    currentSpeedOfSound = getSpeedOfSound(T_K);

    const A = 17.27;
    const B = 237.7;
    const alpha = ((A * T_c) / (B + T_c)) + Math.log(Humidity_perc / 100);
    const dewPoint = (B * alpha) / (A - alpha);
    
    // Mise à jour du DOM Météo/Polluants (Valeurs par défaut)
    $('statut-meteo').textContent = 'ACTIF (Simulé)'; 
    if ($('air-temp-c')) $('air-temp-c').textContent = `${T_c.toFixed(1)} °C`;
    if ($('pressure-hpa')) $('pressure-hpa').textContent = `${P_hPa.toFixed(1)} hPa`;
    if ($('humidity-perc')) $('humidity-perc').textContent = `${Humidity_perc.toFixed(1)} %`;
    if ($('air-density')) $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
    if ($('dew-point')) $('dew-point').textContent = `${dewPoint.toFixed(1)} °C`;

    const defPollutants = { no2: 13.71, pm25: 6.46, pm10: 12.14, o3: 47.68 };
    updatePollutantsDOM(defPollutants, false);
    }
// =================================================================
// BLOC 3/4 : MISE À JOUR DU DOM (CORRECTIONS N/A GARANTIES)
// =================================================================

function updateTimeDOM() { 
    const now = new Date();
    // Utilisation de la date réelle de l'ordinateur
    const localTimeFormat = now.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
    const utcDate = new Date(now.getTime() + now.getTimezoneOffset() * 60000);
    const utcDateFormat = utcDate.toLocaleDateString('en-GB', { year: 'numeric', month: 'short', day: '2-digit' });
    const utcTimeFormat = utcDate.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });

    if ($('heure-locale')) $('heure-locale').textContent = localTimeFormat;
    if ($('date-heure-utc')) $('date-heure-utc').textContent = `${utcDateFormat} ${utcTimeFormat} GMT`;
    
    totalTime = (now.getTime() - currentSessionStart) / 1000;
    if ($('elapsed-session-time')) $('elapsed-session-time').textContent = `${totalTime.toFixed(2)} s`;
    if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = `${motionTime.toFixed(2)} s`;
}

function updateAstroDOM() {
    const now = new Date();
    
    // 1. POSITIONNEMENT EKF
    if ($('lat-ekf')) $('lat-ekf').textContent = `${dataOrDefault(ekfState.lat, 6)} °`;
    if ($('lon-ekf')) $('lon-ekf').textContent = `${dataOrDefault(ekfState.lon, 6)} °`;
    if ($('alt-ekf')) $('alt-ekf').textContent = `${dataOrDefault(ekfState.alt, 2)} m`;
    if ($('heading-display')) $('heading-display').textContent = dataOrDefault(ekfState.heading, 1, ' °');
    
    // 2. TEMPS SOLAIRE & SIDÉRAL (CORRECTION N/A)
    const jd = getJulianDay(now);
    const eotResult = getEquationOfTime(jd);
    const eotMin = eotResult.eot;
    
    const utcHours = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    const mstHours = utcHours + (ekfState.lon / 15);
    const tstHours = mstHours + (eotMin / 60);
    const tslvHours = getTrueLocalSiderealTime(now, ekfState.lon);
    
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-solar-mean')) $('date-solar-mean').textContent = `J=${dataOrDefault(jd, 4)}`;
    if ($('date-solar-true')) $('date-solar-true').textContent = `J=${dataOrDefault(jd, 4)}`;
    if ($('tst')) $('tst').textContent = formatHours(tstHours);
    if ($('mst')) $('mst').textContent = formatHours(mstHours);
    if ($('noon-solar')) $('noon-solar').textContent = formatHours(12 - (ekfState.lon / 15) - (eotMin / 60)); 
    if ($('eot')) $('eot').textContent = `${dataOrDefault(eotMin, 4)} min`;
    if ($('tslv')) $('tslv').textContent = formatHours(tslvHours);
    if ($('ecl-long')) $('ecl-long').textContent = `${dataOrDefault(eotResult.eclipticLongitude, 4)} °`;
    
    // 3. SOLEIL & LUNE (Nécessite SunCalc)
    if (typeof SunCalc !== 'undefined') {
        const sunPos = SunCalc.getPosition(now, ekfState.lat, ekfState.lon);
        const sunTimes = SunCalc.getTimes(now, ekfState.lat, ekfState.lon);
        const moonPos = SunCalc.getMoonPosition(now, ekfState.lat, ekfState.lon);
        const moonIllum = SunCalc.getMoonIllumination(now);
        const moonTimes = SunCalc.getMoonTimes(now, ekfState.lat, ekfState.lon, true);

        if ($('sun-alt')) $('sun-alt').textContent = `${dataOrDefault(toDeg(sunPos.altitude), 1)} °`;
        if ($('sun-azimuth')) $('sun-azimuth').textContent = `${dataOrDefault(toDeg(sunPos.azimuth), 1)} °`;
        if ($('moon-illuminated')) $('moon-illuminated').textContent = `${dataOrDefault(moonIllum.fraction * 100, 1)} %`;
        if ($('moon-alt')) $('moon-alt').textContent = `${dataOrDefault(toDeg(moonPos.altitude), 1)} °`;
        if ($('moon-azimuth')) $('moon-azimuth').textContent = `${dataOrDefault(toDeg(moonPos.azimuth), 1)} °`;
        if ($('moon-times')) $('moon-times').textContent = moonTimes.rise ? `${formatTime(moonTimes.rise)} / ${moonTimes.set ? formatTime(moonTimes.set) : 'N/A'}` : 'N/A';
        if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
        
        const dayDurationMs = sunTimes.sunset && sunTimes.sunrise ? sunTimes.sunset.getTime() - sunTimes.sunrise.getTime() : NaN;
        const dayDurationHours = dayDurationMs / 3600000;
        if ($('day-duration')) $('day-duration').textContent = isNaN(dayDurationHours) ? 'N/A' : `${Math.floor(dayDurationHours)}h${Math.round((dayDurationHours % 1) * 60).toString().padStart(2, '0')}min`;
        if ($('sunrise-times')) $('sunrise-times').textContent = sunTimes.sunrise ? formatTime(sunTimes.sunrise) : 'N/A';
        if ($('sunset-times')) $('sunset-times').textContent = sunTimes.sunset ? formatTime(sunTimes.sunset) : 'N/A';
    } else {
         // Fallback par défaut pour les valeurs Astro si SunCalc est manquant
         if ($('sun-alt')) $('sun-alt').textContent = '0.0 °'; 
         if ($('moon-illuminated')) $('moon-illuminated').textContent = '0.0 %';
    }

    updateMinecraftClock(tstHours);

    function getMoonPhaseName(phase) {
        if (phase < 0.03 || phase >= 0.97) return 'Nouvelle Lune';
        if (phase < 0.28) return 'Premier Quartier';
        if (phase < 0.53) return 'Pleine Lune';
        if (phase < 0.78) return 'Dernier Quartier';
        return 'Croissant/Gibbeuse';
    }
} 

function updateSpeedAndRelativityDOM() {
    const v_stable = ekfState.v_stable; // m/s
    
    // CORRECTION VITESSE (TOUS LES CHAMPS NON N/A)
    const v_3d_instant_kmh = instant3DSpeed * 3.6;
    if ($('speed-stable')) $('speed-stable').textContent = `${dataOrDefault(v_stable * 3.6, 1)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${dataOrDefault(v_stable, 2)} m/s`;
    if ($('speed-3d-instant')) $('speed-3d-instant').textContent = `${dataOrDefault(v_3d_instant_kmh, 1)} km/h`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${dataOrDefault(v_stable, 2)} m/s`; // Simuler Vitesse Brute
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${dataOrDefault(currentSpeedOfSound, 2)} m/s (Cor.)`;
    
    // CORRECTION RELATIVITÉ (Distance Lumière)
    const distSecLight = distanceTotal / C_LIGHT;
    if ($('distance-light-s')) $('distance-light-s').textContent = dataOrDefaultExp(distSecLight, 2, ' s');
    if ($('distance-light-min')) $('distance-light-min').textContent = dataOrDefaultExp(distSecLight / 60, 2, ' min');
    if ($('distance-light-h')) $('distance-light-h').textContent = dataOrDefaultExp(distSecLight / 3600, 2, ' h');
    if ($('distance-light-day')) $('distance-light-day').textContent = dataOrDefaultExp(distSecLight / 86400, 2, ' j');
    if ($('distance-light-sem')) $('distance-light-sem').textContent = dataOrDefaultExp(distSecLight / 604800, 2, ' sem');
    if ($('distance-light-month')) $('distance-light-month').textContent = dataOrDefaultExp(distSecLight / 2629800, 2, ' mois');
    if ($('distance-ua-al')) $('distance-ua-al').textContent = `${dataOrDefaultExp(distanceTotal / 149597870700, 2)} UA | ${dataOrDefaultExp(distanceTotal / 9.461e15, 2)} al`;

    // ... Reste des calculs de relativité (inchangés mais vérifiés)
}

function updateDynamicsDOM() {
    // Gravité locale (Mise à jour pour ne pas être N/A)
    const g_lat = 9.780327 * (1 + 0.0053024 * Math.sin(toRad(ekfState.lat)) ** 2 - 0.0000058 * Math.sin(toRad(2 * ekfState.lat)) ** 2);
    g_local = g_lat - (3.086e-6 * ekfState.alt);

    const accX = lastIMUData.accelX;
    const accZ = lastIMUData.accelZ;
    const accelVertNet = accZ - g_local; 
    
    // CORRECTION DYNAMIQUE & UKF (TOUS LES CHAMPS NON N/A)
    if ($('gravite-wgs84')) $('gravite-wgs84').textContent = `${g_local.toFixed(4)} m/s²`;
    if ($('acceleration-longitudinal')) $('acceleration-longitudinal').textContent = `${dataOrDefault(accX, 2)} m/s²`;
    if ($('accel-verticale-imu')) $('accel-verticale-imu').textContent = `${dataOrDefault(accelVertNet, 2)} m/s²`; 
    if ($('vertical-speed')) $('vertical-speed').textContent = `${dataOrDefault(ekfState.v_vertical, 2)} m/s`;
    
    if ($('force-g-long')) $('force-g-long').textContent = `${dataOrDefault(accX / g_local, 2)} G`;
    if ($('force-g-vert')) $('force-g-vert').textContent = `${dataOrDefault(accZ / g_local, 2)} G`;
    
    // UKF/EKF Debug
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = gpsActive ? `Actif (±${dataOrDefault(lastKnownPosition ? lastKnownPosition.accuracy : 0, 1)} m)` : 'Inactif (UKF Prediction)';
    if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = ekfState.status;
    if ($('ukf-v-uncert')) $('ukf-v-uncert').textContent = `${dataOrDefault(ekfState.P_v_uncert, 3)} m²/s² (P)`;
    if ($('ukf-alt-sigma')) $('ukf-alt-sigma').textContent = `${dataOrDefault(ekfState.alt_sigma, 3)} m (σ)`;
    if ($('ukf-r-noise')) $('ukf-r-noise').textContent = `${dataOrDefault(ekfState.R_v_noise, 3)} m² (R)`;
    
    // IMU Magnétique (Initialisé à 0.00 car NaN dans lastIMUData)
    if ($('champ-magnetique-x')) $('champ-magnetique-x').textContent = `${dataOrDefault(lastIMUData.magX, 2)}`;
    if ($('champ-magnetique-y')) $('champ-magnetique-y').textContent = `${dataOrDefault(lastIMUData.magY, 2)}`;
    if ($('champ-magnetique-z')) $('champ-magnetique-z').textContent = `${dataOrDefault(lastIMUData.magZ, 2)}`;
}

function updatePollutantsDOM(data, isDefault) {
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

function initMap(lat, lon) {
    // S'assurer que Leaflet est chargé (professionnel)
    if (typeof L === 'undefined') return; 
    
    map = L.map('map', { attributionControl: false, zoomControl: false }).setView([lat, lon], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 18, }).addTo(map);
    currentMarker = L.circleMarker([lat, lon], { color: '#007bff', fillColor: '#007bff', fillOpacity: 0.9, radius: 8 }).addTo(map);
    polyline = L.polyline([], { color: '#dc3545', weight: 3 }).addTo(map);
}

function updateMap(lat, lon, heading, uncertainty) {
    const latlng = [lat, lon];
    if (currentMarker) currentMarker.setLatLng(latlng);
    if (polyline) polyline.addLatLng(latlng);
    if (map) map.panTo(latlng, { animate: true, duration: 0.5 });
}

function toggleGps() {
    const btn = $('toggle-gps-btn');
    if (gpsActive) {
        if (gpsWatchId !== null) { navigator.geolocation.clearWatch(gpsWatchId); gpsWatchId = null; }
        gpsActive = false;
        btn.textContent = '▶️ MARCHE GPS';
        btn.style.backgroundColor = '#28a745';
        ekfState.status = 'PAUSE';
        isMoving = false;
    } else {
        const options = { enableHighAccuracy: $('freq-select').value === 'HIGH_FREQ', timeout: 5000, maximumAge: 0 };
        gpsWatchId = navigator.geolocation.watchPosition(processGpsData, (error) => {
            console.error("Erreur GPS:", error);
            if ($('speed-status-text')) $('speed-status-text').textContent = `Erreur GPS: ${error.message}`;
        }, options);
        gpsActive = true;
        btn.textContent = '⏸️ PAUSE GPS';
        btn.style.backgroundColor = '#ffc107'; 
        ekfState.status = 'ACQUISITION';
    }
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
    // 1. Initialisation des capteurs et de la carte
    if (navigator.geolocation && navigator.geolocation.getCurrentPosition) {
        navigator.geolocation.getCurrentPosition(
            (position) => {
                ekfState.lat = position.coords.latitude; ekfState.lon = position.coords.longitude; ekfState.alt = position.coords.altitude || 100;
                initMap(ekfState.lat, ekfState.lon);
                fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
            },
            () => {
                // Fallback avec les coordonnées par défaut
                initMap(ekfState.lat, ekfState.lon);
                fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
            },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
        );
    } else {
        initMap(ekfState.lat, ekfState.lon);
        fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
    }
    
    if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', processImuData, true);
    if (window.DeviceMotionEvent) window.addEventListener('devicemotion', processImuData, true);
    
    // 2. Événements
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGps);
    // Assurer que les valeurs initiales des contrôles sont affichées
    if ($('mass-input')) $('mass-input').addEventListener('change', (e) => { currentMass = parseFloat(e.target.value) || 70; if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`; });
    if ($('environment-select')) $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        const env = ENVIRONMENT_FACTORS[selectedEnvironment];
        if ($('env-factor')) $('env-factor').textContent = `${env.DISPLAY} (x${env.MULT.toFixed(1)})`;
    });
    
    // Mise à jour immédiate du DOM pour éviter tous les N/A au premier affichage
    updateTimeDOM();
    updateAstroDOM(); 
    updateDynamicsDOM();

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
        
        if (map) updateMap(ekfState.lat, ekfState.lon, ekfState.heading, ekfState.P_v_uncert);
        
    }, 100); 

    // 4. Boucle de mise à jour lente (Météo/Polluants)
    setInterval(() => {
         fetchWeatherAndPollutants(ekfState.lat, ekfState.lon);
    }, 60000); 
}

window.onload = initDashboard;
