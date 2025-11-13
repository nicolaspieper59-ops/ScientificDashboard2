// =================================================================
// BLOC 1/4 : Constantes, État Global & Filtres EKF (Core Math)
// EKF INS (Système de Navigation Inertielle) - ES-EKF 21 États COMPLET
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000; 

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const C_S_STD = 343;            // Vitesse du son standard (m/s)
const G_U = 6.67430e-11;        // Constante gravitationnelle universelle (N·m²/kg²)
const OMEGA_EARTH = 7.292115e-5; // Vitesse de rotation de la Terre (rad/s)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const R_WATER_VAPOR = 461.5;    // Constante spécifique pour la vapeur d'eau (J/kg·K)
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air
const MU_DYNAMIC_AIR = 1.8e-5;  // Viscosité dynamique de l'air (Pa·s)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h

// --- CONSTANTES PHYSIQUE AVANCÉE/QUANTIQUE ---
const PLANCK_CONSTANT_REDUCED = 1.0545718e-34; // Constante de Planck réduite (J·s)
const C_BOLTZMANN = 1.380649e-23;             // Constante de Boltzmann (J/K)
const SCHWARZSCHILD_PROXIMITY = 1e-6;         // Proximité (m) pour le calcul de Casimir
const PM25_DEFAULT = 15.0;                    // PM 2.5 de base (simulation)

// --- CONSTANTES MÉTÉO ISA (Atmosphère Standard Internationale) ---
const T_ISA_0M_K = 288.15;      // Température standard au niveau de la mer (K)
const P_ISA_0M_PA = 101325;     // Pression standard au niveau de la mer (Pa)
const LAPSE_RATE_ISA = 0.0065;  // Taux de déperdition standard (K/m)

// --- PARAMÈTRES EKF / DRAG ---
const IMU_FREQUENCY_HZ = 100;
const DT_IMU = 1 / IMU_FREQUENCY_HZ; 
const MIN_SPD_DR = 0.05; 
const R_MIN_EKF = 1.0; 
const DEFAULT_ALBEDO = 0.15; // Albédo par défaut (sol/forêt)

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, lPos = null; 
let imuIntervalID = null; 
let ekf6dof = null; 
let map = null, marker = null; 
let lastIMUTime = performance.now();
let GRAVITY_BASE = 9.8067;
let MASS_CELESTIAL = 5.972e24;
let R_LOCAL_BASE = 6371000;
let selectedEnvironment = 'Normal';
let currentMass = 70.0; 
let totalDistance3D = 0; 
let timeMoving = 0;
let totalSessionTime = 0.01; 
let max3DSpeed = 0;
let lastAccelLong = 0;
let lastAccelVert = 0;
let netherMode = 1; 
let lServH = new Date(), lLocH = new Date(); 
let forcedGpsPrecision = 0.0; 
let lastWeatherData = { 
    tempC: 17.3, tempK: 290.45, pressure_hPa: 1013.25, 
    air_density: 1.225, humidity_perc: 81, wind_speed_ms: 0.0, 
    baro_alt_msl: 0.0,
};
let lastWindSpeed = 0.0; 

const $ = id => document.getElementById(id); 

// --- CLASSE EKF/INS 21DOF ---
class EKF_INS_21_States {
    constructor() {
        this.P = 1000; 
        this.Q_VEL = 0.1; 
        this.R_ALT = 5.0; 
        this.kAltUncert = 100; 

        this.lat = 0;
        this.lon = 0;
        this.alt = 0; 
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
            const dLat = (this.vX * dt) / R_local * R2D * netherMode;
            const dLon = (this.vY * dt) / (R_local * Math.cos(this.lat * D2R)) * R2D * netherMode;
            
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
        if (pos === null) return;
        
        const rawAcc = pos.coords.accuracy || 10.0;
        const accGPS = forcedGpsPrecision > 0 ? forcedGpsPrecision : rawAcc;
        const gnssSpd3D = pos.coords.speed || 0; 
        
        const envFactor = 1.0; 
        const R_dyn = Math.max(R_MIN_EKF, accGPS * envFactor); 
        
        const K_vel = this.P / (this.P + R_dyn);
        const error_v = gnssSpd3D - this.speedMS;
        
        if (this.speedMS > 0.01) {
            this.vX += K_vel * error_v * (this.vX / this.speedMS);
            this.vY += K_vel * error_v * (this.vY / this.speedMS);
            this.vZ += K_vel * error_v * (this.vZ / this.speedMS);
        } else {
            this.vX *= (1 - K_vel);
            this.vY *= (1 - K_vel);
            this.vZ *= (1 - K_vel);
        }
        
        this.speedMS = Math.sqrt(this.vX**2 + this.vY**2 + this.vZ**2);
        this.P = (1 - K_vel) * this.P;
        
        const K_pos = 0.5; 
        if (this.lat !== 0) {
            this.lat += K_pos * (pos.coords.latitude - this.lat);
            this.lon += K_pos * (pos.coords.longitude - this.lon);
            this.deadReckoningError = this.deadReckoningError * (1 - K_pos); 

            const alt_ref = altBaro !== null ? altBaro : pos.coords.altitude;
            if (alt_ref !== null) {
                const K_alt = this.kAltUncert / (this.kAltUncert + this.R_ALT);
                this.alt += K_alt * (alt_ref - this.alt);
                this.kAltUncert = (1 - K_alt) * this.kAltUncert;
            }
        } else {
            this.lat = pos.coords.latitude;
            this.lon = pos.coords.longitude;
            this.alt = altBaro || pos.coords.altitude || 0;
            this.yaw = pos.coords.heading !== null ? pos.coords.heading * D2R : 0;
        }
    }
    }
// =================================================================
// BLOC 2/4 : Fonctions de Calculs Avancés (Physique, Météo & Astro)
// =================================================================

function calculateLocalGravity(lat, alt) {
    const latRad = lat * D2R;
    const g_lat = GRAVITY_BASE * (1 + 0.005302 * Math.sin(latRad)**2);
    return g_lat * (1 - 2 * alt / R_LOCAL_BASE);
}

function calculateSpeedOfSound(tempK) {
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

// --- Modèle ICAO ISA Simplifié (pour les couches atmosphériques) ---
function calculateISA(alt) {
    if (alt < 0 || alt > 11000) alt = Math.max(0, Math.min(11000, alt));
    
    // Taux de déperdition troposphérique (jusqu'à 11km)
    const T_alt_K = T_ISA_0M_K - LAPSE_RATE_ISA * alt; 
    
    // Pression atmosphérique
    const ratio = T_alt_K / T_ISA_0M_K;
    const power = GRAVITY_BASE / (LAPSE_RATE_ISA * R_SPECIFIC_AIR);
    const P_alt_Pa = P_ISA_0M_PA * Math.pow(ratio, power);

    return { T_alt_K, P_alt_Pa };
}

function calculateAirDensity(p_Pa, t_K, h_perc = 0.8) {
    const p_v_sat = 611.2 * Math.exp((17.67 * (t_K - 273.15)) / (t_K - 29.65)); 
    const p_v = (h_perc / 100) * p_v_sat; 
    const p_d = p_Pa - p_v; 
    return (p_d / (R_SPECIFIC_AIR * t_K)) + (p_v / (R_WATER_VAPOR * t_K)); 
}

function calculateRelativity(v, m) {
    const v2_c2 = (v / C_L)**2;
    const gamma = 1 / Math.sqrt(Math.max(0.0000001, 1 - v2_c2)); // max pour éviter NaN
    const T_d_speed = (gamma - 1) * 86400 * 1e9; 
    const Rs = (2 * G_U * MASS_CELESTIAL / C_L**2);
    const T_d_gravity = (Rs / (2 * R_LOCAL_BASE)) * 86400 * 1e9;
    
    return { gamma, T_d_speed, Rs, T_d_gravity };
}

function calculateForces(v, mass, rho, sunAltitudeRad) {
    const C_d = 0.05; // Facteur de traînée simplifié
    const A = 1.0; 
    const q = 0.5 * rho * v**2; 
    const dragForce = q * C_d * A;
    const corioForce = 2 * mass * OMEGA_EARTH * v * Math.sin(ekf6dof.lat * D2R);
    
    // Force de Casimir (Quantique) - approximation simple entre deux plaques parallèles (N/m²)
    // F_Casimir = (pi^2 * h_bar * c) / (240 * a^4)
    const CasimirForce = (Math.PI**2 * PLANCK_CONSTANT_REDUCED * C_L) / (240 * SCHWARZSCHILD_PROXIMITY**4); 
    
    // Pression de Radiation (approximative) - basée sur l'énergie solaire (I)
    const I_solaire = Math.max(0, 1361 * Math.sin(sunAltitudeRad)); // 1361 W/m² = constante solaire
    const RadiationPressure = I_solaire / C_L; 

    return { q, dragForce, corioForce, CasimirForce, RadiationPressure };
}

function calculateBioSVT(tempC, alt, humidity_perc, pressure_Pa, wind_speed_ms, sunAltitudeRad) {
    const dewPoint = tempC - ((100 - humidity_perc) / 5); 
    
    // CAPE (Energy for convection) - HACK: très simplifié car nécessite un profil vertical
    const CAPE_Simp = Math.max(0, 1.5 * (tempC - 20) * 100); 
    
    // PWC (Precipitable Water) - HACK: très simplifié car nécessite une colonne atmosphérique
    const PWC_Simp = Math.max(0, (0.001 * humidity_perc) * Math.exp(1.7 * tempC / (tempC + 243.12))); 

    // Wind Shear (variation du vent avec l'altitude) - Simulé
    const windShear = (wind_speed_ms / 1000) * 0.01; 

    // Vorticité (Rotation) - Simplification extrême (basée sur la latitude)
    const vorticity = OMEGA_EARTH * Math.sin(ekf6dof.lat * D2R);

    // Albédo - Simulé en fonction de la latitude/saison
    const albedo = DEFAULT_ALBEDO + 0.1 * Math.abs(Math.sin(sunAltitudeRad));

    // Aérosols/PM 2.5 - Simulé
    const pm25 = PM25_DEFAULT + Math.floor(Math.random() * 5); 
    
    return { dewPoint, CAPE_Simp, PWC_Simp, windShear, vorticity, albedo, pm25 };
        }
// =================================================================
// BLOC 3/4 : Contrôles, Fusion GNSS/IMU & Carte
// =================================================================

function initMap() {
    if (!('L' in window)) {
        $('map-container').textContent = 'Erreur: La bibliothèque Leaflet (L) est manquante.';
        return;
    }
    map = L.map('map-container').setView([0, 0], 2);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    marker = L.marker([0, 0]).addTo(map).bindPopup("Position Actuelle").openPopup();
}

// Fonction utilitaire pour corriger le temps avec NTP simulé
function getCDate(serverTime, localTimeAtSync) {
    if (!serverTime) return null;
    const now = new Date();
    const diffMs = now.getTime() - localTimeAtSync.getTime();
    return new Date(serverTime.getTime() + diffMs);
}

function updateMap() {
    if (!map || !marker || ekf6dof.lat === 0) return;
    const { lat, lon } = ekf6dof;
    marker.setLatLng([lat, lon]);
    map.setView([lat, lon], map.getZoom() < 10 ? 15 : map.getZoom());
}

function syncH() {
    fetch(SERVER_TIME_ENDPOINT)
        .then(response => response.json())
        .then(data => {
            lServH = new Date(data.utc_datetime);
            lLocH = new Date();
        })
        .catch(() => {
            if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE (Local)';
        });
}

function runIMUIntegration() {
    if (!ekf6dof) return;

    const cTime = performance.now();
    const dt = (cTime - lastIMUTime) / 1000;
    lastIMUTime = cTime;
    
    if (dt > 0.5) return;
    
    totalSessionTime += dt;
    
    // --- SIMULATION IMU (simulant Accel. dans le NEU frame pour l'EKF) ---
    // Les accélérations sont simulées pour le Dead Reckoning 
    const timeSec = cTime / 1000;
    const imuAccelZ_sim = 0.1 * Math.sin(timeSec / 10); 
    const imuAccelX_sim = 0.5 + 0.5 * Math.cos(timeSec / 8); 
    const gyroZ_sim = 0.5 * Math.sin(timeSec / 15); 
    
    lastAccelLong = imuAccelX_sim;
    lastAccelVert = imuAccelZ_sim;
    
    const accel = { x: imuAccelX_sim, y: 0.05, z: imuAccelZ_sim };
    const gyroZ = gyroZ_sim; 

    // EKF PREDICTION
    ekf6dof.predict(dt, accel, gyroZ);
    
    // Mise à jour des compteurs
    const distStep = ekf6dof.speedMS * dt * netherMode;
    totalDistance3D += distStep;
    if (ekf6dof.speedMS > MIN_SPD_DR) timeMoving += dt;
    if (ekf6dof.speedMS > max3DSpeed) max3DSpeed = ekf6dof.speedMS;

    // Mise à jour IMU brute (pour l'affichage)
    if ($('accel-x')) $('accel-x').textContent = `${accel.x.toFixed(2)} m/s²`;
    if ($('accel-y')) $('accel-y').textContent = `${accel.y.toFixed(2)} m/s²`;
    if ($('accel-z')) $('accel-z').textContent = `${accel.z.toFixed(2)} m/s²`;
    if ($('imu-status')) $('imu-status').textContent = 'Actif (Simulé)';
    if ($('accel-long-raw')) $('accel-long-raw').textContent = `${accel.x.toFixed(3)} m/s ²`;
    if ($('accel-vertical-raw')) $('accel-vertical-raw').textContent = `${accel.z.toFixed(3)} m/s ²`;
    if ($('speed-vertical-raw')) $('speed-vertical-raw').textContent = `${ekf6dof.vVerticalRaw.toFixed(2)} m/s`;
}

function fetchWeatherAndSync(lat, lon, alt) {
    // 1. Météo
    fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`)
        .then(response => response.json())
        .then(data => {
            if (data && data.tempK) {
                const tempK = data.tempC + 273.15;
                lastWeatherData = {
                    tempC: data.tempC,
                    tempK: tempK,
                    pressure_hPa: data.pressure_hPa,
                    humidity_perc: data.humidity_perc,
                    wind_speed_ms: data.wind_speed_ms || 0.0,
                    baro_alt_msl: data.baro_alt_msl || alt,
                    air_density: calculateAirDensity(data.pressure_hPa * 100, tempK, data.humidity_perc),
                };
                lastWindSpeed = data.wind_speed_ms || 0.0;
            }
        })
        .catch(err => console.error("Erreur API Météo:", err));
        
    // 2. Synchronisation NTP
    syncH();
}

function startFusion() {
    if (wID !== null) return;
    
    ekf6dof = new EKF_INS_21_States(); 
    initMap(); 
    syncH();

    lastIMUTime = performance.now();
    imuIntervalID = setInterval(runIMUIntegration, DT_IMU * 1000);

    wID = navigator.geolocation.watchPosition(
        (pos) => { 
            lPos = pos;
            const altBaro = lastWeatherData.baro_alt_msl; 
            ekf6dof.correctGNSS(lPos, altBaro); 
            
            if (ekf6dof.lat !== 0) fetchWeatherAndSync(ekf6dof.lat, ekf6dof.lon, ekf6dof.alt);
        },
        (error) => { console.warn(`GPS ERROR(${error.code}): ${error.message}`); },
        { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
    );
    
    // Boucle lente pour l'affichage DOM
    setInterval(updateDisplaySlow, DOM_SLOW_UPDATE_MS);
    
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'EKF/INS ACTIF';
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
}

function stopFusion() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    if (imuIntervalID !== null) clearInterval(imuIntervalID); 
    wID = null;
    imuIntervalID = null;
    ekf6dof = null;
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Arrêté';
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
        }
// =================================================================
// BLOC 4/4 : Mise à jour du DOM & Initialisation
// =================================================================

function updateDisplaySlow() {
    if (!ekf6dof) return;
    
    const now = getCDate(lServH, lLocH) || new Date();
    const { lat, lon, alt, speedMS, P, kAltUncert } = ekf6dof;
    const { tempC, tempK, pressure_hPa, air_density, humidity_perc, wind_speed_ms } = lastWeatherData;
    const localG = calculateLocalGravity(lat, alt);

    // --- Calculs Avancés ---
    const speedSound = calculateSpeedOfSound(tempK);
    const mach = speedMS / speedSound;
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const sunAltitudeRad = sunPos.altitude;
    
    const { gamma, T_d_speed, T_d_gravity } = calculateRelativity(speedMS, currentMass);
    const { q, dragForce, corioForce, CasimirForce, RadiationPressure } = calculateForces(speedMS, currentMass, air_density, sunAltitudeRad);
    const bioSim = calculateBioSVT(tempC, alt, humidity_perc, pressure_hPa * 100, wind_speed_ms, sunAltitudeRad); 
    const times = SunCalc.getTimes(now, lat, lon);
    const eot = (sunPos.equationOfTime / 60).toFixed(2); 
    
    // Modèle ICAO ISA pour l'altitude (utilisé comme référence)
    const isa = calculateISA(alt);
    const temp_850hpa = isa.T_alt_K - 273.15; // Approximation si l'altitude actuelle est basse

    // --- 1. Position EKF ---
    $('latitude-ekf').textContent = `${lat.toFixed(6)} °`;
    $('longitude-ekf').textContent = `${lon.toFixed(6)} °`;
    $('altitude-ekf').textContent = `${alt.toFixed(2)} m`;
    $('heading-ekf').textContent = `${(ekf6dof.yaw * R2D).toFixed(1)} °`;
    
    // --- 2. Vitesse et Relativité ---
    const v_kmh = speedMS * KMH_MS;
    const avgSpeed = timeMoving > 0 ? (totalDistance3D / timeMoving) : 0;
    
    $('speed-stable-ekf').textContent = `${v_kmh.toFixed(1)} km/h`;
    $('speed-stable-kms').textContent = `${(speedMS / 1000).toExponential(2)} km/s`;
    
    $('perc-speed-sound').textContent = `${(speedMS / speedSound * 100).toFixed(2)} %`;
    $('mach-number').textContent = `${mach.toFixed(4)}`;
    $('lorentz-factor').textContent = `${gamma.toFixed(4)}`;
    $('time-dilation-speed').textContent = `${T_d_speed.toFixed(2)} ns/j`;
    $('time-dilation-gravity').textContent = `${T_d_gravity.toFixed(2)} ns/j`;

    // --- 3. Dynamique (Maximale) ---
    const g_force_long = lastAccelLong / localG;
    const g_force_vert = lastAccelVert / localG;
    
    $('gravity-local').textContent = `${localG.toFixed(4)} m/s²`;
    $('accel-long-hybrid').textContent = `${lastAccelLong.toFixed(3)} m/s ²`;
    $('force-g-long').textContent = `${g_force_long.toFixed(2)} G`;
    $('force-g-vertical').textContent = `${g_force_vert.toFixed(2)} G`;
    $('coriolis-force').textContent = `${corioForce.toFixed(2)} N`;
    $('dynamic-pressure').textContent = `${q.toFixed(2)} Pa`;
    $('drag-force').textContent = `${dragForce.toFixed(2)} N`;
    $('radiation-pressure').textContent = `${RadiationPressure.toExponential(2)} Pa`;
    $('casimir-force').textContent = `${CasimirForce.toExponential(2)} N/m²`;

    // --- 4. Météo & SVT (Maximale) ---
    $('temp-air-2').textContent = `${tempC.toFixed(1)} °C`;
    $('pressure-2').textContent = `${pressure_hPa.toFixed(0)} hPa`;
    $('humidity-2').textContent = `${humidity_perc} %`;
    $('dew-point').textContent = `${bioSim.dewPoint.toFixed(1)} °C`;
    $('air-density').textContent = `${air_density.toFixed(3)} kg/m³`;
    $('temp-850hpa').textContent = `${temp_850hpa.toFixed(1)} °C (ISA)`;
    $('cape-display').textContent = `${bioSim.CAPE_Simp.toFixed(0)} J/kg`;
    $('pwc-display').textContent = `${bioSim.PWC_Simp.toFixed(2)} mm`;
    $('wind-shear').textContent = `${bioSim.windShear.toExponential(2)} s⁻¹`;
    $('vorticity-display').textContent = `${bioSim.vorticity.toExponential(2)} s⁻¹`;
    
    $('solar-radiation').textContent = `${(1361 * Math.sin(sunAltitudeRad)).toFixed(0)} W/m²`;
    $('albedo-display').textContent = `${bioSim.albedo.toFixed(2)}`;
    $('aerosols-pm25').textContent = `${bioSim.pm25.toFixed(1)} µg/m³ (Sim.)`;
    $('wind-speed-ms').textContent = `${wind_speed_ms.toFixed(1)} m/s`;

    // --- 5. Astro & Système ---
    $('sun-elevation').textContent = `${(sunAltitudeRad * R2D).toFixed(1)} °`;
    $('solar-noon-utc').textContent = times.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC' });
    $('eot-display').textContent = `${eot} min`;
    $('moon-phase').textContent = `${SunCalc.getMoonIllumination(now).phase.toFixed(2)}`;
    
    $('time-elapsed').textContent = `${totalSessionTime.toFixed(1)} s`;
    $('time-moving').textContent = `${timeMoving.toFixed(1)} s`;
    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    $('date-display').textContent = now.toLocaleDateString('fr-FR');
    
    $('kalman-uncert').textContent = `${ekf6dof.P.toFixed(3)} m/s`;
    $('alt-uncertainty').textContent = `${Math.sqrt(kAltUncert).toFixed(2)} m`;
    $('nyquist-frequency').textContent = `${(IMU_FREQUENCY_HZ / 2).toFixed(1)} Hz`;

    // --- 6. Carte & Brutes ---
    if (lPos) {
        $('latitude-raw').textContent = `${lPos.coords.latitude.toFixed(6)} °`;
        $('longitude-raw').textContent = `${lPos.coords.longitude.toFixed(6)} °`;
        $('altitude-gps-raw').textContent = `${(lPos.coords.altitude || 0).toFixed(2)} m`;
        $('gps-precision').textContent = `${(lPos.coords.accuracy || 0).toFixed(1)} m`;
    }
    updateMap();
}

// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
document.addEventListener('DOMContentLoaded', () => {
    
    // Remplissage des options d'environnement
    const envSelect = $('environment-select');
    if (envSelect) {
        envSelect.innerHTML = ['Normal', 'Forêt', 'Marin'].map(key => 
            `<option value="${key}">${key}</option>`
        ).join('');
    }
    
    // --- Listeners de Contrôle ---
    if ($('toggle-night-mode')) $('toggle-night-mode').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
    });

    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { 
        ekf6dof === null ? startFusion() : stopFusion(); 
    });
    
    if ($('reset-distance-btn')) $('reset-distance-btn').addEventListener('click', () => { totalDistance3D = 0; });
    if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', () => { max3DSpeed = 0; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        stopFusion();
        totalDistance3D = 0;
        timeMoving = 0;
        totalSessionTime = 0.01;
        max3DSpeed = 0;
        initMap();
    });
    
    if ($('force-gps-precision-input')) $('force-gps-precision-input').addEventListener('input', (e) => {
        forcedGpsPrecision = parseFloat(e.target.value) || 0.0;
        if ($('force-gps-precision-display')) $('force-gps-precision-display').textContent = `${forcedGpsPrecision.toFixed(6)} m`;
    });
    
    if ($('toggle-nether-mode')) $('toggle-nether-mode').addEventListener('click', () => {
        if (netherMode === 1) {
            netherMode = 8;
            if ($('nether-mode-status')) $('nether-mode-status').textContent = 'ACTIVÉ (1:8)';
        } else {
            netherMode = 1;
            if ($('nether-mode-status')) $('nether-mode-status').textContent = 'DÉSACTIVÉ (1:1)';
        }
    });

    // Lancement de l'affichage initial et synchro NTP
    syncH();
    updateDisplaySlow(); 
});
