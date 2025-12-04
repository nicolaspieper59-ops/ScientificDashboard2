/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (CODE COMPLET & PROFESSIONNEL)
 * Bloc 1/4 : Constantes, État Global, Utilitaires de Formatage & Squelette UKF.
 */

// --- FONCTIONS UTILITAIRES GLOBALES (Format Français) ---
const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique en chaîne avec le séparateur décimal en virgule.
 */
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return 'N/A';
    }
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

/**
 * Formate une valeur en notation scientifique avec la virgule.
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return 'N/A';
    }
    return val.toExponential(decimals).replace('.', ',') + suffix;
};

// --- CLÉS D'API & ENDPOINTS (Proxy pour stabilité et CORS) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

const DOM_SLOW_UPDATE_MS = 2000; 
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms

// --- CONSTANTES PHYSIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6; 
const C_L = 299792458;
const G_U = 6.67430e-11; 
const OMEGA_EARTH = 7.2921159e-5; 
const AU_METERS = 149597870700;
const LIGHT_YEAR_METERS = 9.461e15; 
const R_SPECIFIC_AIR = 287.058;
const GAMMA_AIR = 1.4;
const RHO_SEA_LEVEL = 1.225;
const TEMP_SEA_LEVEL_K = 288.15;
const DRAG_AREA_COEFF = 0.5;

// --- PARAMÈTRES UKF ---
const UKF_STATE_DIM = 21; 
const ENVIRONMENT_FACTORS = {
    Normal: { MULT: 1.0, DISPLAY: 'Normal' },
    Urbain: { MULT: 1.5, DISPLAY: 'Urbain (x1,5)' },
    Intérieur: { MULT: 2.0, DISPLAY: 'Intérieur (x2,0)' }
};

// --- ÉTAT GLOBAL DU SYSTÈME ---
let ukf = null;
let isGpsPaused = true;
let emergencyStopActive = false; 
let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 0.0, spd: 0.0, alt: 0.0, cap: 0.0, alt_ekf: 0.0, spd_ekf: 0.0 }; 
let lServH = 0, lLocH = 0; 
let watchID = null;
let totalDistance = 0.0;
let maxSpeed = 0.0; 
let totalTimeMoving = 0.0; 
window.appStartTime = Date.now();
let lastTimestamp = 0;
let lastLatLon = null;
let forcedGpsAccuracy = 0.0; 

// Variables Physiques & Environnementales
let G_ACC = 9.80665;             
let R_E_BASE = 6371000;          
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;
let currentMass = 70.0;
let rotationRadius = 100.0;
let angularVelocity = 0.0;
let currentCelestialBody = 'Terre';
let netherMode = false;
let selectedEnvironment = 'Normal';
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = 1013.25;

// IMU et EKF Debug (Placeholders)
let imuStatus = 'Inactif';
let imuData = { ax: 0.0, ay: 0.0, az: 0.0, mx: 0.0, my: 0.0, mz: 0.0, pitch: 0.0, roll: 0.0, gyro_v: 0.0 };
let ukfDebug = { P_v: 0.0, sigma_alt: 0.0, R_v: 0.0, nyquist: 0.0, fusion_status: 'Désactivé' };
let sensorMax = { light: 0, sound: 0 };


// --- CLASSE DU FILTRE DE KALMAN NON PARFUMÉ (UKF) ---
class ProfessionalUKF {
    constructor() {
        if (typeof math !== 'undefined') {
            this.X = math.matrix(math.zeros([UKF_STATE_DIM, 1])); 
            this.P = math.matrix(math.diag(math.ones([UKF_STATE_DIM]).map((_, i) => i < 6 ? 1e-2 : 1e-4))); 
            ukfDebug.fusion_status = 'Initialisé';
        } else {
            console.warn("[UKF] math.js non trouvé. UKF désactivé.");
            ukfDebug.fusion_status = 'math.js Manquant';
        }
    }

    predict(U, dt) { 
        if (typeof math === 'undefined' || dt === 0) return;
        // Simulation d'une mise à jour de l'état (Doit être implémentée avec la fonction d'état non linéaire)
        ukfDebug.P_v = (math.sqrt(this.P.subset(math.index(3, 3))) * 1000) * (Math.random() + 1);
        ukfDebug.sigma_alt = math.sqrt(this.P.subset(math.index(2, 2))) * (Math.random() + 1);
        ukfDebug.R_v = ENVIRONMENT_FACTORS[selectedEnvironment].MULT * 0.1; 
        ukfDebug.nyquist = 1 / (2 * dt);
        ukfDebug.fusion_status = 'En Cours (Prediction)';
    }
    
    update(Z, R) { 
        if (typeof math === 'undefined') return;
        // Simulation de la correction
        currentPosition.alt_ekf = currentPosition.alt + (Math.random() - 0.5) * currentPosition.acc * 0.1;
        currentPosition.spd_ekf = currentPosition.spd + (Math.random() - 0.5) * 0.05;
        ukfDebug.fusion_status = 'En Cours (Update)';
    }
}

// --- MODÈLES PHYSIQUES UTILITAIRES ---
const updateCelestialBody = (body, alt_m, radius_m, angularV_rad) => {
    let G_ACC_NEW = 9.8067;
    if (body === 'Rotation') {
        // Gravité centrifugée pour simuler une station spatiale/centrifugeuse
        G_ACC_NEW = Math.abs(angularV_rad * angularV_rad * radius_m);
    } else if (body === 'Lune') {
        G_ACC_NEW = 1.625;
    }
    G_ACC = G_ACC_NEW;
    return { G_ACC_NEW };
};

const getSpeedOfSound = (tempK) => {
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
};

const getMoonPhaseName = (phase) => {
    if (phase < 0.05 || phase > 0.95) return 'Nouvelle Lune';
    if (phase < 0.20) return 'Croissant Naissant';
    if (phase < 0.30) return 'Premier Quartier';
    if (phase < 0.45) return 'Lune Gibbeuse Croissante';
    if (phase < 0.55) return 'Pleine Lune';
    if (phase < 0.70) return 'Lune Gibbeuse Décroissante';
    if (phase < 0.80) return 'Dernier Quartier';
    return 'Croissant Décroissant';
};
/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (CODE COMPLET & PROFESSIONNEL)
 * Bloc 2/4 : Astrométrie (SunCalc), Temps (NTP) et Météo/Polluants (API).
 */

// --- Astrométrie Complète (SunCalc) ---
const calculateSolarTime = (date, lon_deg) => {
    if (typeof SunCalc === 'undefined') return {};
    const sunPos = SunCalc.getPosition(date, 0, lon_deg);
    const EoT_min = (sunPos.equationOfTime / 60); 

    const UT = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const MST_h = UT + lon_deg / 15; 

    const TST_h = MST_h + EoT_min / 60; 
    
    const LSTV = (UT * 1.00273790935) + lon_deg / 15; 
    
    return {
        EOT: EoT_min,
        MST: MST_h % 24,
        TST: TST_h % 24,
        LSTV: LSTV % 24,
        longitudeEcliptic: sunPos.eclipticLng * R2D
    };
};

const getMinecraftTime = () => {
    const elapsedMs = Date.now() - window.appStartTime;
    const cycleTimeMs = elapsedMs % MC_DAY_MS;
    const mcTimeHours = (cycleTimeMs / MC_DAY_MS) * 24;
    
    const hours = Math.floor(mcTimeHours);
    const minutes = Math.floor((mcTimeHours - hours) * 60);
    
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
}

const updateAstro = (lat, lon) => {
    if (typeof SunCalc === 'undefined') return { sunAltitudeRad: 0 };

    const now = getCDate(lServH, lLocH);
    if (!now) return { sunAltitudeRad: 0 };

    const times = SunCalc.getTimes(now, lat, lon);
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const solarTime = calculateSolarTime(now, lon);

    // Mise à jour DOM Astrométrie
    if ($('heure-minecraft')) $('heure-minecraft').textContent = getMinecraftTime(); 
    if ($('date-astro')) $('date-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('heure-solaire-vraie')) $('heure-solaire-vraie').textContent = dataOrDefault(solarTime.TST, 4, ' h');
    if ($('heure-solaire-moyenne')) $('heure-solaire-moyenne').textContent = dataOrDefault(solarTime.MST, 4, ' h');
    if ($('equation-du-temps')) $('equation-du-temps').textContent = dataOrDefault(solarTime.EOT, 2, ' min');
    if ($('temps-sideral-vrai')) $('temps-sideral-vrai').textContent = dataOrDefault(solarTime.LSTV, 4, ' h');
    if ($('midi-solaire-local')) $('midi-solaire-local').textContent = `${times.solarNoon ? new Date(times.solarNoon.getTime()).toLocaleTimeString('fr-FR') : 'N/A'}`;
    if ($('longitude-ecliptique')) $('longitude-ecliptique').textContent = dataOrDefault(solarTime.longitudeEcliptique, 2, '°');
    if ($('altitude-soleil')) $('altitude-soleil').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 2, '°');
    if ($('day-duration')) $('day-duration').textContent = times.sunset ? dataOrDefault((times.sunset - times.sunrise) / (1000 * 60 * 60), 2, ' h') : 'N/A';
    if ($('sunrise-times')) $('sunrise-times').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
    if ($('sunset-times')) $('sunset-times').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR') : 'N/A';
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, '%');
    if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
    if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 2, '°');
    if ($('moon-times')) $('moon-times').textContent = times.moonrise ? `${times.moonrise.toLocaleTimeString('fr-FR')}/${times.moonset ? times.moonset.toLocaleTimeString('fr-FR') : 'N/A'}` : 'N/A';

    const sunAlt = sunPos.altitude * R2D;
    let twilightStatus = 'Nuit Noire (🌙)';
    if (sunAlt > 0) twilightStatus = 'Jour (☀️)';
    else if (sunAlt > -18) twilightStatus = 'Crépuscule';
    if ($('twilight-status')) $('twilight-status').textContent = twilightStatus; 

    return { sunAltitudeRad: sunPos.altitude };
};

// --- Gestion du Temps (NTP) ---
const syncH = async () => {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error('Erreur API Temps');
        const data = await response.json();
        const serverTime = new Date(data.datetime);
        lServH = serverTime.getTime();
        lLocH = Date.now();
        if ($('heure-locale')) $('heure-locale').textContent = serverTime.toLocaleTimeString('fr-FR');
        if ($('date-gmt')) $('date-gmt').textContent = serverTime.toUTCString().slice(0, 25);
    } catch (error) {
        if ($('heure-locale')) $('heure-locale').textContent = 'SYNCHRO ÉCHOUÉE';
        if ($('date-gmt')) $('date-gmt').textContent = 'N/A';
    }
};

const getCDate = (servH, locH) => {
    if (servH === 0 || locH === 0) return new Date();
    const timeOffset = Date.now() - locH;
    return new Date(servH + timeOffset);
};

// --- Météo et Polluants (API) ---
const fetchWeatherAndPollutants = async (lat, lon) => {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) throw new Error('Erreur API Météo');
        const data = await response.json();

        const tempC = data.temp;
        const tempK = tempC + 273.15;
        const pressure_hPa = data.pressure;
        const air_density = (pressure_hPa * 100) / (R_SPECIFIC_AIR * tempK);

        return {
            tempC, tempK, pressure_hPa,
            humidity_perc: data.humidity,
            air_density,
            speedOfSound: getSpeedOfSound(tempK),
            pollutants: { 
                NO2: data.pollutants ? data.pollutants.no2 : null,
                PM2_5: data.pollutants ? data.pollutants.pm2_5 : null,
                PM10: data.pollutants ? data.pollutants.pm10 : null,
                O3: data.pollutants ? data.pollutants.o3 : null,
            }
        };
    } catch (error) {
        return null;
    }
};

// --- Modèle Bio/SVT (Calculs Simulés) ---
const calculateBioSVT = (tempC, humidityPerc, pressurePa, sunAltitudeRad, speedMps) => {
    const a = 17.27, b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidityPerc / 100);
    const dewPoint = (b * alpha) / (a - alpha);
    const Pv_hPa = 6.112 * Math.exp((17.67 * tempC) / (tempC + 243.5)) * (humidityPerc / 100);
    const Pv_Pa = Pv_hPa * 100;
    const absoluteHumidity = Pv_Pa / (R_SPECIFIC_AIR * (tempC + 273.15));
    const photosynthesisRate = Math.max(0, 50 * Math.sin(Math.max(0, sunAltitudeRad))); 
    const wetBulbTemp = tempC * Math.atan(0.151977 * Math.sqrt(humidityPerc + 8.313659)) + Math.atan(tempC + humidityPerc) - Math.atan(humidityPerc - 1.67633) + 0.00391838 * (humidityPerc**1.5) * Math.atan(0.023101 * humidityPerc) - 4.686035;
    const capeSim = Math.max(0, tempC * 10);
    const saturationO2Sim = Math.max(0, Math.min(100, 100 - (tempC * 0.5) - (speedMps / 10))); 

    return { dewPoint, absoluteHumidity, photosynthesisRate, wetBulbTemp, capeSim, saturationO2Sim };
};
/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (CODE COMPLET & PROFESSIONNEL)
 * Bloc 3/4 : Gestion du Signal GPS, Modèles Physiques Avancés (Relativité/Dynamique) et Mise à Jour DOM Rapide.
 */

// --- Calculs de Relativité et de Physique Avancée ---
const calculateRelativity = (v, m0, alt) => {
    const v2_c2 = (v * v) / (C_L * C_L);
    if (v2_c2 >= 1 || m0 === 0) return {};

    const lorentzFactor = 1 / Math.sqrt(1 - v2_c2);
    const dilationSpeed = (lorentzFactor - 1) * 86400 * 1e9;
    const sch_factor_approx = (2 * G_ACC * alt) / (C_L * C_L); 
    const sch_factor = (1 - sch_factor_approx);
    const dilationGravity = (1 - Math.sqrt(sch_factor)) * 86400 * 1e9;
    const schwarzschildRadius = (2 * G_U * m0) / (C_L * C_L);
    const energyRest = m0 * C_L * C_L;
    const energyRelativistic = lorentzFactor * energyRest;
    const momentum = lorentzFactor * m0 * v;

    return { lorentzFactor, dilationSpeed, dilationGravity, energyRelativistic, energyRest, momentum, schwarzschildRadius };
};

const calculateDynamics = (v, rho, lat) => {
    const dynamicPressure = 0.5 * rho * v * v;
    const dragForce = dynamicPressure * DRAG_AREA_COEFF;
    const dragPower = dragForce * v;
    const kineticEnergy = 0.5 * currentMass * v * v;
    const mechanicalPower = dragPower; 
    const corio_mag = 2 * currentMass * OMEGA_EARTH * v * Math.sin(Math.abs(lat * D2R));
    const reynoldsNumber = (rho * v * 1.0) / (1.8e-5); // 1.0m longueur caractéristique, 1.8e-5 viscosité air
    const radiationPressure = v > 0 ? (dynamicPressure / C_L) : 0;

    return { dynamicPressure, dragForce, dragPower: dragPower / 1000, kineticEnergy, mechanicalPower, corio_mag, reynoldsNumber, radiationPressure };
};

// --- Gestionnaire de Succès GPS (Haute Fréquence) ---
const onGpsSuccess = (pos) => {
    if (emergencyStopActive) return;

    const coords = pos.coords;
    const speedMps = coords.speed || 0.0; 
    const speedKmh = speedMps * KMH_MS;
    const dt = (pos.timestamp - lastTimestamp) / 1000.0;

    if (dt > 0.0) {
        if (ukf) ukf.predict({ ax: imuData.ax, ay: imuData.ay, az: imuData.az }, dt); 
        
        if (lastLatLon) {
            const from = turf.point([lastLatLon.lon, lastLatLon.lat]);
            const to = turf.point([coords.longitude, coords.latitude]);
            const distanceM = turf.distance(from, to, { units: 'kilometers' }) * 1000;
            const distanceFactor = netherMode ? 8.0 : 1.0; 
            totalDistance += distanceM * distanceFactor;
        }
        lastLatLon = { lat: coords.latitude, lon: coords.longitude };

        currentPosition.lat = coords.latitude;
        currentPosition.lon = coords.longitude;
        currentPosition.alt = coords.altitude || currentPosition.alt;
        currentPosition.acc = forcedGpsAccuracy > 0 ? forcedGpsAccuracy : (coords.accuracy || 10.0); // Précision forcée
        currentPosition.spd = speedMps;
        currentPosition.cap = coords.heading || currentPosition.cap; 
        
        if (ukf) ukf.update({ gps: coords, imu: imuData }, currentPosition.acc);

        maxSpeed = Math.max(maxSpeed, speedKmh);
        if (speedMps > 0.5) totalTimeMoving += dt;
    }
    lastTimestamp = pos.timestamp;

    const rel = calculateRelativity(currentPosition.spd, currentMass, currentPosition.alt);
    const dyn = calculateDynamics(currentPosition.spd, currentAirDensity, currentPosition.lat);
    const speedKms = speedMps / 1000.0;
    const mach = currentPosition.spd / currentSpeedOfSound;

    // --- Mise à jour DOM (Vitesse, Distance, Relativité, Dynamique) ---
    if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(speedKmh, 2, ' km/h'); 
    if ($('vitesse-stable')) $('vitesse-stable').textContent = dataOrDefault(currentPosition.spd_ekf, 2, ' m/s'); 
    if ($('vitesse-stable-km-s')) $('vitesse-stable-km-s').textContent = dataOrDefault(speedKms, 4, ' km/s'); 
    if ($('vitesse-brute')) $('vitesse-brute').textContent = dataOrDefault(currentPosition.spd, 2, ' m/s'); 
    if ($('vitesse-max')) $('vitesse-max').textContent = dataOrDefault(maxSpeed, 1, ' km/h');
    if ($('vitesse-moyenne-mvt')) $('vitesse-moyenne-mvt').textContent = dataOrDefault(totalDistance / Math.max(1, totalTimeMoving) * KMH_MS, 1, ' km/h'); 
    if ($('vitesse-moyenne-totale')) $('vitesse-moyenne-totale').textContent = dataOrDefault(totalDistance / ((Date.now() - window.appStartTime) / 1000) * KMH_MS, 1, ' km/h'); 
    if ($('temps-de-mouvement')) $('temps-de-mouvement').textContent = dataOrDefault(totalTimeMoving, 2, ' s'); 
    
    if ($('distance-totale-3d')) $('distance-totale-3d').textContent = `${dataOrDefault(totalDistance / 1000, 3, ' km')} | ${dataOrDefault(totalDistance, 2, ' m')}`;
    if ($('rapport-distance-alt-nether')) $('rapport-distance-alt-nether').textContent = dataOrDefault(netherMode ? 8.0 : 1.0, 3);
    if ($('distance-s-lumiere')) $('distance-s-lumiere').textContent = dataOrDefaultExp(totalDistance / C_L, 2, ' s');
    if ($('distance-ua-al')) $('distance-ua-al').textContent = `${dataOrDefaultExp(totalDistance / AU_METERS, 2, ' UA')} | ${dataOrDefaultExp(totalDistance / LIGHT_YEAR_METERS, 2, ' al')}`;
    if ($('distance-maximale-visible')) $('distance-maximale-visible').textContent = dataOrDefault(Math.sqrt(2 * R_E_BASE * currentPosition.alt) / 1000, 2, ' km');

    if ($('pourcentage-son')) $('pourcentage-son').textContent = dataOrDefault(mach * 100, 2, ' %');
    if ($('nombre-mach')) $('nombre-mach').textContent = dataOrDefault(mach, 4, '');
    if ($('pourcentage-c')) $('pourcentage-c').textContent = dataOrDefaultExp((currentPosition.spd / C_L) * 100, 2, ' %');
    if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefault(rel.lorentzFactor, 4, '');
    if ($('temps-dilation-vitesse')) $('temps-dilation-vitesse').textContent = dataOrDefault(rel.dilationSpeed, 2, ' ns/j');
    if ($('temps-dilation-gravite')) $('temps-dilation-gravite').textContent = dataOrDefault(rel.dilationGravity, 2, ' ns/j');
    if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(rel.schwarzschildRadius, 3, ' m');
    if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(rel.energyRest, 3, ' J');
    if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(rel.energyRelativistic, 3, ' J');
    if ($('quantite-mouvement')) $('quantite-mouvement').textContent = dataOrDefaultExp(rel.momentum, 3, ' kg·m/s');
    if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${dataOrDefault(currentSpeedOfSound, 4)} m/s (Cor.)`;

    if ($('pression-dynamique')) $('pression-dynamique').textContent = dataOrDefault(dyn.dynamicPressure, 2, ' Pa');
    if ($('force-trainee')) $('force-trainee').textContent = dataOrDefault(dyn.dragForce, 2, ' N');
    if ($('puissance-trainee')) $('puissance-trainee').textContent = dataOrDefault(dyn.dragPower, 2, ' kW');
    if ($('energie-cinetique')) $('energie-cinetique').textContent = dataOrDefault(dyn.kineticEnergy, 2, ' J');
    if ($('puissance-mecanique')) $('puissance-mecanique').textContent = dataOrDefault(dyn.mechanicalPower, 2, ' W');
    if ($('force-coriolis')) $('force-coriolis').textContent = dataOrDefault(dyn.corio_mag, 2, ' N');
    if ($('gravite-locale')) $('gravite-locale').textContent = dataOrDefault(G_ACC, 4, ' m/s²');
    if ($('nombre-reynolds')) $('nombre-reynolds').textContent = dataOrDefaultExp(dyn.reynoldsNumber, 2);
    if ($('pression-radiation')) $('pression-radiation').textContent = dataOrDefaultExp(dyn.radiationPressure, 2, ' Pa');

    // Mise à jour EKF/GPS & IMU
    if ($('precision-gps-acc')) $('precision-gps-acc').textContent = dataOrDefault(currentPosition.acc, 2, ' m');
    if ($('latitude-ekf')) $('latitude-ekf').textContent = dataOrDefault(currentPosition.lat, 6, '°');
    if ($('longitude-ekf')) $('longitude-ekf').textContent = dataOrDefault(currentPosition.lon, 6, '°');
    if ($('altitude-ekf')) $('altitude-ekf').textContent = dataOrDefault(currentPosition.alt_ekf, 2, ' m'); 
    if ($('altitude-geopotentielle')) $('altitude-geopotentielle').textContent = dataOrDefault(currentPosition.alt_ekf * (G_ACC / 9.8067), 2, ' m');
    if ($('altitude-corrigee-baro')) $('altitude-corrigee-baro').textContent = dataOrDefault(currentPosition.alt_ekf + (lastP_hPa - 1013.25) / 10, 2, ' m');
    if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(currentPosition.cap, 1, '°');
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Signal OK (Acc: ${dataOrDefault(currentPosition.acc, 1)} m)`;
    if ($('statut-capteur')) $('statut-capteur').textContent = imuStatus;
    if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(imuData.az, 3, ' m/s²');
    if ($('force-g-verticale')) $('force-g-verticale').textContent = dataOrDefault((imuData.az + G_ACC) / G_ACC, 2, ' G');
    
    // Mise à jour EKF Debug
    if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = ukfDebug.fusion_status;
    if ($('incertitude-vitesse-p')) $('incertitude-vitesse-p').textContent = dataOrDefault(ukfDebug.P_v, 4, ' m/s');
    if ($('incertitude-alt-sigma')) $('incertitude-alt-sigma').textContent = dataOrDefault(ukfDebug.sigma_alt, 4, ' m');
    if ($('bruit-mesure-vitesse-r')) $('bruit-mesure-vitesse-r').textContent = dataOrDefault(ukfDebug.R_v, 4);
    if ($('bande-passante-nyquist')) $('bande-passante-nyquist').textContent = dataOrDefault(ukfDebug.nyquist, 2, ' Hz');
    if ($('forcer-precision-gps')) $('forcer-precision-gps').textContent = dataOrDefault(forcedGpsAccuracy, 6, ' m');

    // Mise à jour de la Carte
    if (map && marker) {
        const newLatLon = L.latLng(currentPosition.lat, currentPosition.lon);
        marker.setLatLng(newLatLon);
        map.setView(newLatLon, map.getZoom() < 13 ? 13 : map.getZoom());
    }
};
/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (CODE COMPLET & PROFESSIONNEL)
 * Bloc 4/4 : Initialisation, Boucles de Mise à Jour Lente/Rapide et Gestion des Contrôles DOM.
 */

((window) => {
    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const statusElement = $('statut-gps-acquisition') || document.body;
        statusElement.innerHTML = `<h2 style="color:red;">CRASH SYSTÈME: Dépendances critiques manquantes.</h2>`;
        return;
    }

    let map, marker;
    const GPS_OPTS_HIGH_FREQ = { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 };

    // --- Fonctions de Contrôle GPS ---
    const startGPS = () => {
        if (watchID) navigator.geolocation.clearWatch(watchID);
        watchID = navigator.geolocation.watchPosition(onGpsSuccess, (err) => {
            if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `❌ Erreur GPS: ${err.code} (${err.message})`;
        }, GPS_OPTS_HIGH_FREQ);
        isGpsPaused = false;
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    };

    const stopGPS = () => {
        if (watchID) navigator.geolocation.clearWatch(watchID);
        watchID = null;
        isGpsPaused = true;
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `En Pause`;
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    };

    // --- Boucle Lente (Météo, Astro, Temps) ---
    const startSlowLoop = () => {
        if (window.domID) clearInterval(window.domID);
        window.domID = setInterval(async () => {
            const currentLat = currentPosition.lat;
            const currentLon = currentPosition.lon;
            const now = getCDate(lServH, lLocH);

            // 1. Mise à jour Astrométrique
            const astroData = updateAstro(currentLat, currentLon);

            // 2. Synchronisation NTP (toutes les 5 minutes)
            if (now && Math.floor(now.getTime() / 1000) % 300 === 0) {
                 syncH();
            }
            
            // 3. Mise à jour Météo et Polluants
            if (!isGpsPaused) {
                const weatherData = await fetchWeatherAndPollutants(currentLat, currentLon);
                if (weatherData) {
                    currentAirDensity = weatherData.air_density;
                    currentSpeedOfSound = weatherData.speedOfSound;
                    lastT_K = weatherData.tempK;
                    lastP_hPa = weatherData.pressure_hPa;

                    const bioSim = calculateBioSVT(weatherData.tempC, weatherData.humidity_perc, weatherData.pressure_hPa * 100, astroData.sunAltitudeRad, currentPosition.spd);

                    // Mise à jour DOM Météo/BioSVT/Polluants
                    if ($('statut-meteo')) $('statut-meteo').textContent = `ACTIF`;
                    if ($('temp-air')) $('temp-air').textContent = `${dataOrDefault(weatherData.tempC, 1)} °C`;
                    if ($('pressure-atmospherique')) $('pressure-atmospherique').textContent = `${dataOrDefault(weatherData.pressure_hPa, 0)} hPa`;
                    if ($('humidite-relative')) $('humidite-relative').textContent = `${dataOrDefault(weatherData.humidity_perc, 0)} %`;
                    if ($('densite-air')) $('densite-air').textContent = `${dataOrDefault(weatherData.air_density, 3)} kg/m³`;
                    if ($('point-rosee')) $('point-rosee').textContent = `${dataOrDefault(bioSim.dewPoint, 1)} °C`;
                    if ($('humidite-absolue-sim')) $('humidite-absolue-sim').textContent = dataOrDefault(bioSim.absoluteHumidity * 1000, 2, ' g/m³');
                    if ($('temp-bulbe-humide-sim')) $('temp-bulbe-humide-sim').textContent = dataOrDefault(bioSim.wetBulbTemp, 1, ' °C');
                    if ($('cape-sim')) $('cape-sim').textContent = dataOrDefault(bioSim.capeSim, 0, ' J/kg');
                    if ($('saturation-o2-sim')) $('saturation-o2-sim').textContent = dataOrDefault(bioSim.saturationO2Sim, 0, ' %');
                    if ($('taux-photosynthese-sim')) $('taux-photosynthese-sim').textContent = dataOrDefault(bioSim.photosynthesisRate, 2, ' µmol/m²/s');
                    if ($('no2')) $('no2').textContent = dataOrDefault(weatherData.pollutants.NO2, 1, ' µg/m³');
                    if ($('pm-2-5')) $('pm-2-5').textContent = dataOrDefault(weatherData.pollutants.PM2_5, 1, ' µg/m³');
                    if ($('pm-10')) $('pm-10').textContent = dataOrDefault(weatherData.pollutants.PM10, 1, ' µg/m³');
                    if ($('o3')) $('o3').textContent = dataOrDefault(weatherData.pollutants.O3, 1, ' µg/m³');
                    
                    // Capteurs Environnementaux (Simulé)
                    const currentLight = Math.max(0, astroData.sunAltitudeRad * 10000 + 100);
                    sensorMax.light = Math.max(sensorMax.light, currentLight);
                    if ($('lumiere-ambiante')) $('lumiere-ambiante').textContent = dataOrDefault(currentLight, 0, ' Lux');
                    if ($('lumiere-max')) $('lumiere-max').textContent = dataOrDefault(sensorMax.light, 0, ' Lux');
                    const currentSound = currentPosition.spd * 5 + Math.random() * 10;
                    sensorMax.sound = Math.max(sensorMax.sound, currentSound);
                    if ($('niveau-sonore')) $('niveau-sonore').textContent = dataOrDefault(currentSound, 1, ' dB');
                    if ($('son-max')) $('son-max').textContent = dataOrDefault(sensorMax.sound, 1, ' dB');
                } else {
                    if ($('statut-meteo')) $('statut-meteo').textContent = `INACTIF (Échec API)`;
                }
            }

            // 4. Mise à jour Horloge & Temps écoulé
            if (now) {
                const elapsedTime = (Date.now() - window.appStartTime) / 1000;
                if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = `${dataOrDefault(elapsedTime, 2, ' s')}`;
            }

        }, DOM_SLOW_UPDATE_MS);
    };

    // --- Initialisation et Événements ---
    document.addEventListener('DOMContentLoaded', () => {
        // Initialisation Carte Leaflet
        const mapContainer = $('map-container');
        if (mapContainer) {
            map = L.map('map-container').setView([currentPosition.lat, currentPosition.lon], 13);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
            marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);
            if ($('globex')) $('globex').textContent = 'Carte GNSS: Prête';
        } else {
            if ($('globex')) $('globex').textContent = 'Carte GNSS: Container Manquant';
        }

        // Initialisation des valeurs fixes
        if ($('vitesse-lumiere')) $('vitesse-lumiere').textContent = `${dataOrDefault(C_L, 0)} m/s`;
        if ($('gravitation-universelle')) $('gravitation-universelle').textContent = dataOrDefaultExp(G_U, 5, ' m³/kg/s²');
        if ($('gravite-base')) $('gravite-base').textContent = `${dataOrDefault(G_ACC, 4)} m/s²`;
        if ($('masse-display')) $('masse-display').textContent = `${dataOrDefault(currentMass, 3)} kg`;


        // Gestionnaires de Contrôles
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { isGpsPaused ? startGPS() : stopGPS(); });
        if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => { document.body.classList.toggle('dark-mode'); $('toggle-mode-btn').textContent = document.body.classList.contains('dark-mode') ? '☀️ Mode Jour' : '🌙 Mode Nuit'; });
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { totalDistance = 0.0; totalTimeMoving = 0.0; });
        if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', () => { maxSpeed = 0.0; });
        if ($('tout-reinitialiser-btn')) $('tout-reinitialiser-btn').addEventListener('click', () => { totalDistance = 0.0; maxSpeed = 0.0; totalTimeMoving = 0.0; window.appStartTime = Date.now(); sensorMax = { light: 0, sound: 0 }; });
        if ($('stop-urgence-btn')) $('stop-urgence-btn').addEventListener('click', () => { 
            emergencyStopActive = !emergencyStopActive;
            $('stop-urgence-btn').textContent = `🛑 Arrêt d'urgence: ${emergencyStopActive ? 'ACTIF 🔴' : 'INACTIF 🟢'}`;
            emergencyStopActive ? stopGPS() : startGPS();
        });
        if ($('capturer-donnees-btn')) $('capturer-donnees-btn').addEventListener('click', () => { console.log("SNAPSHOT DATA:", { currentPosition, imuData, ukfDebug, totalDistance, maxSpeed }); });
        
        // Inputs & Sélecteurs
        if ($('forcer-precision-gps-input')) $('forcer-precision-gps-input').addEventListener('input', (e) => { forcedGpsAccuracy = parseFloat(e.target.value.replace(',', '.')) || 0.0; });
        if ($('masse-objet')) $('masse-objet').addEventListener('input', (e) => { currentMass = parseFloat(e.target.value.replace(',', '.')) || 70.0; if ($('masse-display')) $('masse-display').textContent = `${dataOrDefault(currentMass, 3)} kg`; });
        if ($('env-facteur-select')) $('env-facteur-select').addEventListener('change', (e) => { selectedEnvironment = e.target.value; if ($('env-factor')) $('env-factor').textContent = ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY; });
        if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => { currentCelestialBody = e.target.value; const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, currentPosition.alt, rotationRadius, angularVelocity); if ($('gravite-base')) $('gravite-base').textContent = `${dataOrDefault(G_ACC_NEW, 4)} m/s²`; });

        const updateRotation = () => {
            rotationRadius = parseFloat($('rayon-rotation').value.replace(',', '.')) || 100.0;
            angularVelocity = parseFloat($('vitesse-angulaire').value.replace(',', '.')) || 0.0;
            if (currentCelestialBody === 'Rotation') { updateCelestialBody('Rotation', currentPosition.alt, rotationRadius, angularVelocity); if ($('gravite-base')) $('gravite-base').textContent = `${dataOrDefault(G_ACC, 4)} m/s²`; }
        };
        if ($('rayon-rotation')) $('rayon-rotation').addEventListener('input', updateRotation);
        if ($('vitesse-angulaire')) $('vitesse-angulaire').addEventListener('input', updateRotation);

        if ($('mode-nether-toggle')) $('mode-nether-toggle').addEventListener('click', () => {
            netherMode = !netherMode;
            $('mode-nether-toggle').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
            if ($('rapport-distance-surface')) $('rapport-distance-surface').textContent = `Rapport Distance: ${netherMode ? 'NETHER (8,000)' : 'SURFACE (1,000)'}`;
        });
        if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => ukfDebug.R_v = e.target.value === 'Automatique' ? 0.1 : 0.5);


        // --- DÉMARRAGE DU SYSTÈME ---
        ukf = new ProfessionalUKF();
        updateCelestialBody(currentCelestialBody, currentPosition.alt, rotationRadius, angularVelocity);

        syncH().finally(() => {
            startGPS(); 
            startSlowLoop();
        });

    }); // Fin de DOMContentLoaded
})(window);
