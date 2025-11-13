// =================================================================
// PARTIE 1/2 : CONSTANTES, ÉTAT GLOBAL & MATHÉMATIQUES
// =================================================================

// --- CONFIGURATION & API ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 1000; 

// --- CONSTANTES PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse lumière (m/s)
const C_S_STD = 343;            // Vitesse son (m/s)
const G_U = 6.67430e-11;        // Constante gravitationnelle
const OMEGA_EARTH = 7.292115e-5; // Vitesse rotation Terre
const R_SPECIFIC_AIR = 287.058; // Constante gaz air sec
const R_WATER_VAPOR = 461.5;    // Constante gaz vapeur eau
const GAMMA_AIR = 1.4;          // Indice adiabatique
const MU_DYNAMIC_AIR = 1.8e-5;  // Viscosité dynamique
const R_E_BASE = 6371000;       // Rayon Terre
const KMH_MS = 3.6;             // Conversion m/s -> km/h

// --- CONSTANTES AVANCÉES (Quantique / ISA) ---
const PLANCK_CONSTANT_REDUCED = 1.0545718e-34; 
const C_BOLTZMANN = 1.380649e-23;             
const SCHWARZSCHILD_PROXIMITY = 1e-6;         
const PM25_DEFAULT = 15.0;                    
const T_ISA_0M_K = 288.15;      
const P_ISA_0M_PA = 101325;     
const LAPSE_RATE_ISA = 0.0065;  

// --- PARAMÈTRES SYSTÈME ---
const IMU_FREQUENCY_HZ = 100;
const DT_IMU = 1 / IMU_FREQUENCY_HZ; 
const MIN_SPD_DR = 0.05; 
const R_MIN_EKF = 1.0; 
const DEFAULT_ALBEDO = 0.15;

// --- DONNÉES & ENVIRONNEMENT ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, M: 5.972e24 },
    'MOON': { G: 1.62, R: 1737400, M: 7.347e22 },
    'MARS': { G: 3.71, R: 3389500, M: 6.417e23 },
    'ROTATING': { G: 0.0, R: R_E_BASE, M: 0 }
};

const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DRAG_C: 0.5, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DRAG_C: 0.8, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DRAG_C: 0.9, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DRAG_C: 0.9, DISPLAY: 'Métal/Bâtiment' },
};

// --- VARIABLES D'ÉTAT GLOBALES (Partagées) ---
let wID = null, domID = null, lPos = null;
let lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, timeMoving = 0;
let kSpd = 0, kUncert = 1000; 
let lServH = new Date(), lLocH = new Date(); 
let lastFSpeed = 0; 
let kAlt = null, kAltUncert = 10;  
let emergencyStopActive = false;
let netherMode = false; 
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; 
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let G_ACC = CELESTIAL_DATA['EARTH'].G;
let MASS_CELESTIAL = CELESTIAL_DATA['EARTH'].M;
let R_LOCAL_BASE = CELESTIAL_DATA['EARTH'].R;
let lastWeatherData = { 
    tempC: 20, tempK: 293.15, pressure_hPa: 1013, 
    humidity_perc: 50, air_density: 1.225, wind_speed_ms: 0 
}; 
let sunAltitudeRad = 0;
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let map, marker, circle;
let GPS_ACCURACY_OVERRIDE = 0;
let ekf6dof = null; 
let imuIntervalID = null; 
let totalDistance3D = 0;
let totalSessionTime = 0.01;
let max3DSpeed = 0;
let lastAccelLong = 0;

// --- CLASSE EKF INS 21 ÉTATS ---
class EKF_INS_21_States {
    constructor() {
        this.P = 1000; 
        this.Q_VEL = 0.1; 
        this.R_ALT = 5.0; 
        this.kAltUncert = 100; 
        this.lat = 0; this.lon = 0; this.alt = 0; 
        this.speedMS = 0; 
        this.vX = 0; this.vY = 0; this.vZ = 0; 
        this.yaw = 0.0; 
        this.deadReckoningError = 0;
        this.vVerticalRaw = 0; 
    }

    predict(dt, accel, gyroZ) {
        if (dt <= 0) return;
        this.yaw = (this.yaw + gyroZ * dt * D2R + 2 * Math.PI) % (2 * Math.PI); 
        const localG = calculateLocalGravity(this.lat, this.alt);
        
        this.vX += accel.x * dt;
        this.vY += accel.y * dt;
        this.vZ += (accel.z - localG) * dt; 
        
        if (this.lat !== 0) {
            const R_local = R_LOCAL_BASE + this.alt;
            const dLat = (this.vX * dt) / R_local * R2D;
            const dLon = (this.vY * dt) / (R_local * Math.cos(this.lat * D2R)) * R2D;
            this.lat += dLat;
            this.lon += dLon;
            this.alt += this.vZ * dt;
        }
        this.speedMS = Math.sqrt(this.vX**2 + this.vY**2 + this.vZ**2);
        this.P += this.Q_VEL * dt;
        this.deadReckoningError += this.speedMS * this.Q_VEL * dt;
        this.vVerticalRaw = accel.z * dt;
    }
    
    correctGNSS(pos, altBaro) {
        if (!pos) return;
        const rawAcc = pos.coords.accuracy || 10.0;
        const accGPS = GPS_ACCURACY_OVERRIDE > 0 ? GPS_ACCURACY_OVERRIDE : rawAcc;
        const gnssSpd3D = pos.coords.speed || 0; 
        const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
        const R_dyn = Math.max(R_MIN_EKF, accGPS * envFactor); 
        
        const K_vel = this.P / (this.P + R_dyn);
        const error_v = gnssSpd3D - this.speedMS;
        
        if (this.speedMS > 0.01) {
            this.vX += K_vel * error_v * (this.vX / this.speedMS);
            this.vY += K_vel * error_v * (this.vY / this.speedMS);
            this.vZ += K_vel * error_v * (this.vZ / this.speedMS);
        } else {
            this.vX *= (1 - K_vel); this.vY *= (1 - K_vel); this.vZ *= (1 - K_vel);
        }
        this.speedMS = Math.sqrt(this.vX**2 + this.vY**2 + this.vZ**2);
        this.P = (1 - K_vel) * this.P;
        
        const K_pos = 0.5; 
        if (this.lat !== 0) {
            this.lat += K_pos * (pos.coords.latitude - this.lat);
            this.lon += K_pos * (pos.coords.longitude - this.lon);
            this.deadReckoningError *= (1 - K_pos); 
            const alt_ref = altBaro !== null ? altBaro : pos.coords.altitude;
            if (alt_ref !== null) {
                const K_alt = this.kAltUncert / (this.kAltUncert + this.R_ALT);
                this.alt += K_alt * (alt_ref - this.alt);
                this.kAltUncert *= (1 - K_alt);
            }
        } else {
            this.lat = pos.coords.latitude;
            this.lon = pos.coords.longitude;
            this.alt = altBaro || pos.coords.altitude || 0;
            this.yaw = pos.coords.heading ? pos.coords.heading * D2R : 0;
        }
    }
}

// --- FONCTIONS MATHÉMATIQUES AVANCÉES ---

function calculateLocalGravity(lat, alt) {
    const latRad = lat * D2R;
    const g_lat = GRAVITY_BASE * (1 + 0.005302 * Math.sin(latRad)**2);
    return g_lat * (1 - 2 * alt / R_LOCAL_BASE);
}

function calculateRelativity(v, m) {
    const v2_c2 = (v / C_L)**2;
    const gamma = 1 / Math.sqrt(Math.max(0.0000001, 1 - v2_c2)); 
    const T_d_speed = (gamma - 1) * 86400 * 1e9; 
    const Rs = (2 * G_U * MASS_CELESTIAL / C_L**2);
    const T_d_gravity = (Rs / (2 * R_LOCAL_BASE)) * 86400 * 1e9;
    const E0 = m * C_L**2;
    const E = gamma * E0;
    return { gamma, T_d_speed, Rs, T_d_gravity, E0, E };
}

function calculateForces(v, mass, rho, sunAltitudeRad) {
    const C_d = ENVIRONMENT_FACTORS[selectedEnvironment].DRAG_C; 
    const A = 1.0; 
    const q = 0.5 * rho * v**2; 
    const dragForce = q * C_d * A;
    const corioForce = 2 * mass * OMEGA_EARTH * v * Math.sin(ekf6dof.lat * D2R);
    const CasimirForce = (Math.PI**2 * PLANCK_CONSTANT_REDUCED * C_L) / (240 * SCHWARZSCHILD_PROXIMITY**4); 
    const I_solaire = Math.max(0, 1361 * Math.sin(sunAltitudeRad)); 
    const RadiationPressure = I_solaire / C_L; 
    return { q, dragForce, corioForce, CasimirForce, RadiationPressure };
}

function calculateBioSVT(tempC, alt, humidity_perc, pressure_Pa, wind_speed_ms, sunAltitudeRad) {
    const dewPoint = tempC - ((100 - humidity_perc) / 5); 
    const windChill = (tempC < 15 && wind_speed_ms > 1.3) ? 13.12 + 0.6215 * tempC - 11.37 * Math.pow(wind_speed_ms * 3.6, 0.16) + 0.3965 * tempC * Math.pow(wind_speed_ms * 3.6, 0.16) : tempC;
    
    const O2_LEVEL = 20.9; 
    const CO2_LEVEL = 420 + Math.random() * 5; 
    const OZONE = 300 + 50 * Math.sin(alt / 10000); 
    const PH = 6.5 + 0.5 * Math.random(); 
    const NOISE = 40 + wind_speed_ms * 5; 
    const NDVI = Math.min(0.9, 0.2 + 0.7 * (Math.sin(alt / 5000) + 1) / 2); 
    const CAPE = Math.max(0, 1.5 * (tempC - 20) * 100); 

    return { dewPoint, windChill, O2_LEVEL, CO2_LEVEL, OZONE, PH, NOISE, NDVI, CAPE };
}

function calculateSpeedOfSound(tempK) {
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

function calculateISA(alt) {
    if (alt < 0 || alt > 11000) alt = Math.max(0, Math.min(11000, alt));
    const T_alt_K = T_ISA_0M_K - LAPSE_RATE_ISA * alt; 
    const ratio = T_alt_K / T_ISA_0M_K;
    const power = GRAVITY_BASE / (LAPSE_RATE_ISA * R_SPECIFIC_AIR);
    const P_alt_Pa = P_ISA_0M_PA * Math.pow(ratio, power);
    return { T_alt_K, P_alt_Pa };
}

function calculateAirDensity(p, t, h) {
    const pv = (h/100) * 6.1078 * Math.pow(10, (7.5*(t-273.15))/(237.3+(t-273.15))) * 100;
    const pd = p - pv;
    return (pd / (R_SPECIFIC_AIR * t)) + (pv / (R_WATER_VAPOR * t));
}

function calculateMinecraftTime(sunAltitudeRad) {
    let solarFraction = (sunAltitudeRad / (2 * Math.PI) + 0.25) % 1; 
    let ticks = Math.round(solarFraction * 24000) % 24000;
    ticks = (ticks + 18000) % 24000;
    const hours = Math.floor(ticks / 1000) % 24; 
    const minutes = Math.floor((ticks % 1000) * 0.06); 
    const h_str = String(hours).padStart(2, '0');
    const m_str = String(minutes).padStart(2, '0');
    return { ticks, time: `${h_str}:${m_str}` };
    }
// =================================================================
// PARTIE 2/2 : GESTION DES BOUCLES, DOM & EVENTS
// =================================================================

const $ = id => document.getElementById(id);

// --- GESTION CARTE & UTILS ---
function initMap() {
    if (typeof L === 'undefined') return;
    map = L.map('map').setView([48.8566, 2.3522], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
    marker = L.marker([0, 0]).addTo(map);
    circle = L.circle([0, 0], { radius: 10 }).addTo(map);
}

function updateMap(lat, lon, acc) {
    if (!map || !marker) return;
    const newLatLng = L.latLng(lat, lon);
    marker.setLatLng(newLatLng);
    circle.setLatLng(newLatLng).setRadius(acc || 10);
    map.panTo(newLatLng);
}

function getCDate(serv, loc) {
    if (!serv) return new Date();
    return new Date(serv.getTime() + (Date.now() - loc.getTime()));
}

// --- GESTION DES BOUCLES ET CAPTEURS ---

function imuMotionHandler(event) {
    if (event.accelerationIncludingGravity) {
        real_accel_x = event.accelerationIncludingGravity.x || 0;
        real_accel_y = event.accelerationIncludingGravity.y || 0;
        real_accel_z = event.accelerationIncludingGravity.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif";
    }
}

function startFusion() {
    if (wID !== null) return;
    
    ekf6dof = new EKF_INS_21_States(); 
    initMap(); 
    if (window.DeviceMotionEvent) window.addEventListener('devicemotion', imuMotionHandler, true);

    lastIMUTime = performance.now();
    imuIntervalID = setInterval(runIMUIntegration, DT_IMU * 1000);

    wID = navigator.geolocation.watchPosition(
        (pos) => { 
            lPos = pos;
            const altBaro = lastWeatherData.baro_alt_msl; 
            ekf6dof.correctGNSS(lPos, altBaro); 
            fetchWeatherAndSync(ekf6dof.lat, ekf6dof.lon);
        },
        (error) => console.warn("GPS Error"),
        { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
    );
    
    setInterval(updateDisplaySlow, DOM_SLOW_UPDATE_MS);
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'EKF/INS ACTIF';
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
}

function stopFusion() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    if (imuIntervalID !== null) clearInterval(imuIntervalID); 
    window.removeEventListener('devicemotion', imuMotionHandler, true);
    wID = null;
    ekf6dof = null;
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Arrêté';
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
}

function runIMUIntegration() {
    if (!ekf6dof) return;
    const cTime = performance.now();
    const dt = (cTime - lastIMUTime) / 1000;
    lastIMUTime = cTime;
    if (dt > 0.5) return;
    totalSessionTime += dt;

    const accel = { x: real_accel_x, y: real_accel_y, z: real_accel_z }; 
    const gyroZ = 0; 

    ekf6dof.predict(dt, accel, gyroZ);
    
    const V = ekf6dof.speedMS;
    const distStep = V * dt * (netherMode ? 8 : 1);
    totalDistance3D += distStep;
    if (V > MIN_SPD_DR) timeMoving += dt;
    if (V * KMH_MS > max3DSpeed) max3DSpeed = V * KMH_MS;
    
    lastAccelLong = (V - lastFSpeed) / dt;
    lastFSpeed = V;

    if ($('imu-accel-x')) $('imu-accel-x').textContent = real_accel_x.toFixed(2) + ' m/s²';
    if ($('imu-accel-y')) $('imu-accel-y').textContent = real_accel_y.toFixed(2) + ' m/s²';
    if ($('imu-accel-z')) $('imu-accel-z').textContent = real_accel_z.toFixed(2) + ' m/s²';
}

function fetchWeatherAndSync(lat, lon) {
    fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`)
        .then(r => r.json())
        .then(d => {
            if (d && d.tempK) {
                lastWeatherData = {
                    tempC: d.tempC, tempK: d.tempK,
                    pressure_hPa: d.pressure_hPa,
                    humidity_perc: d.humidity_perc,
                    air_density: calculateAirDensity(d.pressure_hPa*100, d.tempK, d.humidity_perc),
                    wind_speed_ms: d.wind_speed_ms || 0,
                    baro_alt_msl: d.baro_alt_msl || 0
                };
            }
        }).catch(e => {});
    
    fetch(SERVER_TIME_ENDPOINT).then(r => r.json()).then(d => {
        lServH = new Date(d.utc_datetime); lLocH = new Date();
    }).catch(() => {});
}

// --- MISE À JOUR DE L'AFFICHAGE (SLOW LOOP) ---

function updateDisplaySlow() {
    if (!ekf6dof) return;
    const now = getCDate(lServH, lLocH);
    const { lat, lon, alt, speedMS, P, kAltUncert, deadReckoningError } = ekf6dof;
    const { tempC, tempK, pressure_hPa, air_density, humidity_perc, wind_speed_ms } = lastWeatherData;
    const localG = calculateLocalGravity(lat, alt);

    const sunPos = typeof SunCalc !== 'undefined' ? SunCalc.getPosition(now, lat, lon) : {altitude:0, azimuth:0};
    sunAltitudeRad = sunPos.altitude;
    const times = typeof SunCalc !== 'undefined' ? SunCalc.getTimes(now, lat, lon) : {};
    const moon = typeof SunCalc !== 'undefined' ? SunCalc.getMoonIllumination(now) : {phase:0};

    const phy = calculateRelativity(speedMS, currentMass);
    const forces = calculateForces(speedMS, currentMass, air_density, sunAltitudeRad);
    const bio = calculateBioSVT(tempC, alt, humidity_perc, pressure_hPa*100, wind_speed_ms, sunAltitudeRad);
    
    const speedSound = calculateSpeedOfSound(tempK);
    const mach = speedMS / speedSound;
    
    // Update DOM
    if ($('lat-display')) $('lat-display').textContent = lat.toFixed(6) + ' °';
    if ($('lon-display')) $('lon-display').textContent = lon.toFixed(6) + ' °';
    if ($('alt-display')) $('alt-display').textContent = alt.toFixed(2) + ' m';
    if ($('geopotential-alt')) $('geopotential-alt').textContent = (alt * 9.80665 / localG).toFixed(2) + ' m';
    if ($('heading-display')) $('heading-display').textContent = (ekf6dof.yaw * R2D).toFixed(1) + ' °';
    
    if ($('speed-stable')) $('speed-stable').textContent = (speedMS * KMH_MS).toFixed(1) + ' km/h';
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = speedMS.toFixed(2) + ' m/s';
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = (speedMS/1000).toExponential(2) + ' km/s';
    if ($('speed-max')) $('speed-max').textContent = (max3DSpeed * KMH_MS).toFixed(1) + ' km/h';
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = ((totalDistance3D / (timeMoving||1)) * KMH_MS).toFixed(1) + ' km/h';
    
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = (mach * 100).toFixed(2) + ' %';
    if ($('perc-speed-c')) $('perc-speed-c').textContent = ((speedMS/C_L)*100).toExponential(2) + ' %';
    if ($('mach-number')) $('mach-number').textContent = mach.toFixed(4);
    if ($('lorentz-factor')) $('lorentz-factor').textContent = phy.gamma.toFixed(5);
    
    if ($('distance-total-km')) $('distance-total-km').textContent = (totalDistance3D/1000).toFixed(3) + ' km';
    if ($('distance-light-s')) $('distance-light-s').textContent = (totalDistance3D/C_L).toExponential(2) + ' s';
    if ($('distance-cosmic')) $('distance-cosmic').textContent = (totalDistance3D/AU_METERS).toExponential(2) + ' UA';
    
    if ($('gravity-local')) $('gravity-local').textContent = localG.toFixed(4) + ' m/s²';
    if ($('accel-long')) $('accel-long').textContent = lastAccelLong.toFixed(3) + ' m/s²';
    if ($('force-g-long')) $('force-g-long').textContent = (forces.dragForce / (currentMass*localG)).toFixed(2) + ' G';
    if ($('coriolis-force')) $('coriolis-force').textContent = forces.corioForce.toExponential(2) + ' N';
    if ($('kinetic-energy')) $('kinetic-energy').textContent = (0.5*currentMass*speedMS*speedMS).toExponential(2) + ' J';
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = forces.q.toFixed(2) + ' Pa';
    if ($('casimir-force')) $('casimir-force').textContent = forces.CasimirForce.toExponential(2) + ' N';
    if ($('radiation-pressure')) $('radiation-pressure').textContent = forces.RadiationPressure.toExponential(2) + ' Pa';
    if ($('time-dilation-speed')) $('time-dilation-speed').textContent = phy.T_d_speed.toFixed(3) + ' ns/j';
    if ($('grav-dilation')) $('grav-dilation').textContent = phy.T_d_gravity.toFixed(3) + ' ns/j';
    if ($('energy-relativistic')) $('energy-relativistic').textContent = phy.E.toExponential(2) + ' J';
    if ($('energy-rest-mass')) $('energy-rest-mass').textContent = phy.E0.toExponential(2) + ' J';
    if ($('Rs-object')) $('Rs-object').textContent = phy.Rs.toExponential(2) + ' m';
    if ($('momentum')) $('momentum').textContent = (phy.gamma * currentMass * speedMS).toFixed(2) + ' kg·m/s';

    if ($('temp-air-2')) $('temp-air-2').textContent = tempC.toFixed(1) + ' °C';
    if ($('pressure-2')) $('pressure-2').textContent = pressure_hPa.toFixed(0) + ' hPa';
    if ($('humidity-2')) $('humidity-2').textContent = humidity_perc + ' %';
    if ($('dew-point')) $('dew-point').textContent = bio.dewPoint.toFixed(1) + ' °C';
    if ($('air-density')) $('air-density').textContent = air_density.toFixed(3) + ' kg/m³';
    if ($('temp-feels-like')) $('temp-feels-like').textContent = bio.windChill.toFixed(1) + ' °C';
    if ($('wind-speed-ms')) $('wind-speed-ms').textContent = wind_speed_ms.toFixed(1) + ' m/s';
    
    if ($('o2-level')) $('o2-level').textContent = bio.O2_LEVEL + ' %';
    if ($('co2-level')) $('co2-level').textContent = bio.CO2_LEVEL.toFixed(0) + ' ppm';
    if ($('ozone-conc')) $('ozone-conc').textContent = bio.OZONE.toFixed(0) + ' DU';
    if ($('ph-level')) $('ph-level').textContent = bio.PH.toFixed(1);
    if ($('noise-level')) $('noise-level').textContent = bio.NOISE.toFixed(1) + ' dB';
    if ($('ndvi-index')) $('ndvi-index').textContent = bio.NDVI.toFixed(2);
    if ($('cape-sim')) $('CAPE-sim').textContent = bio.CAPE.toFixed(0) + ' J/kg';

    // Astro
    const minecraftTime = calculateMinecraftTime(sunAltitudeRad);
    if ($('time-minecraft')) $('time-minecraft').textContent = minecraftTime.time;
    
    if ($('tst')) $('tst').textContent = (12 + (lon/15) + ((sunPos.azimuth*R2D)/15) % 24).toFixed(2) + ' h';
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunAltitudeRad * R2D).toFixed(1) + ' °';
    if ($('noon-solar')) $('noon-solar').textContent = times.solarNoon ? times.solarNoon.toLocaleTimeString() : 'N/A';
    if ($('moon-phase-name')) $('moon-phase-name').textContent = (moon.phase * 100).toFixed(0) + ' %';

    if ($('kalman-uncert')) $('kalman-uncert').textContent = P.toFixed(3);
    if ($('speed-error-perc')) $('speed-error-perc').textContent = (Math.sqrt(P)/(speedMS||1)*100).toFixed(1) + '%';
    if ($('gps-precision')) $('gps-precision').textContent = (lPos ? lPos.coords.accuracy : 0).toFixed(1) + ' m';
    
    // Horloge Visuelle
    const sunDeg = (sunAltitudeRad * R2D);
    if ($('sun-element')) $('sun-element').style.transform = `rotate(${sunDeg}deg) translateX(35px) rotate(-${sunDeg}deg)`;
    if ($('moon-element')) $('moon-element').style.transform = `rotate(${sunDeg + 180}deg) translateX(35px) rotate(-${sunDeg + 180}deg)`;
    
    updateMap(lat, lon, lPos ? lPos.coords.accuracy : 10);
}

document.addEventListener('DOMContentLoaded', () => {
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => ekf6dof ? stopFusion() : startFusion());
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => { emergencyStopActive = !emergencyStopActive; $('emergency-stop-btn').classList.toggle('active'); });
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => document.body.classList.toggle('dark-mode'));
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { netherMode = !netherMode; $('mode-nether').textContent = netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { totalDistance3D = 0; maxSpd = 0; timeMoving = 0; });
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        GRAVITY_BASE = CELESTIAL_DATA[currentCelestialBody].G;
        R_LOCAL_BASE = CELESTIAL_DATA[currentCelestialBody].R;
        MASS_CELESTIAL = CELESTIAL_DATA[currentCelestialBody].M;
        $('gravity-base').textContent = GRAVITY_BASE.toFixed(4) + ' m/s²';
    });
    
    syncH();
});
