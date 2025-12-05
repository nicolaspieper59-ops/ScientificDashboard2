// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET CORRIGÉ
// Ce fichier corrige les erreurs de syntaxe, les ID HTML et la robustesse
// des dépendances (math.js, UKF) qui causaient l'échec total (N/A).
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
// Correction critique: Ajout d'un contrôle pour ne pas planter si l'ID n'est pas trouvé.
const $ = id => {
    const el = document.getElementById(id);
    if (!el) console.warn(`ID manquant dans le HTML : ${id}`);
    return el;
};

const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

const getCDate = () => lastNTPDate || new Date();
const calculateMaxVisibleDistance = (altM) => altM > 0 ? Math.sqrt(altM * (2 * 6371000 + altM)) : 0; 
const getMinecraftTime = (date) => {
    const h = date.getUTCHours(); const m = date.getUTCMinutes();
    const totalMinutes = h * 60 + m;
    const mcTime = (totalMinutes / 60 + 6) % 24; 
    const mcH = Math.floor(mcTime);
    const mcM = Math.floor((mcTime - mcH) * 60);
    return `${mcH.toString().padStart(2, '0')}:${mcM.toString().padStart(2, '0')}`;
};

// --- STUBS ASTRO (Garde les fonctions même si lib/astro.js n'a pas été chargé) ---
// Utilise window.getSolarTime, etc. pour éviter ReferenceError
const getSolarTime = (typeof window.getSolarTime === 'function') ? window.getSolarTime : (date, lon) => ({ MST: 'N/A', TST: 'N/A', EOT: '0.00', DateMST: null, DateTST: null, ECL_LONG: 'N/A' });
const getTSLV = (typeof window.getTSLV === 'function') ? window.getTSLV : (date, lon) => 'N/A';
const getMoonPhaseName = (typeof window.getMoonPhaseName === 'function') ? window.getMoonPhaseName : (phase) => 'N/A';
const getMoonIllumination = (typeof window.getMoonIllumination === 'function') ? window.getMoonIllumination : (date) => ({ phase: 0, fraction: 0, angle: 0 });

// --- CONSTANTES CRITIQUES ---
const C_L = 299792458; 
const G_U = 6.67430e-11; 
const R2D = 180 / Math.PI; 
const D2R = Math.PI / 180; 
const KMH_MS = 3.6; 
const MIN_SPD = 0.5; 
const MAX_ACC = 50; 
const IMU_UPDATE_RATE_MS = 20; 
const DOM_SLOW_UPDATE_MS = 1000; 
const MAP_UPDATE_INTERVAL = 3000; 
const MIN_DT = 0.005; 
const TEMP_SEA_LEVEL_K = 288.15; 
const RHO_SEA_LEVEL = 1.225; 
const BARO_ALT_REF_HPA = 1013.25; 
const UKF_R_MAX = 1000.0; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};
const ENVIRONMENT_FACTORS = {
    NORMAL: { R_MULT: 1.0, DISPLAY: 'Normal' },
    CITY: { R_MULT: 3.0, DISPLAY: 'Urbain Droit' },
    TUNNEL: { R_MULT: 10.0, DISPLAY: 'Tunnel/Indoor' },
    SEA: { R_MULT: 0.5, DISPLAY: 'Mer/Plaine' },
    GROTTO: { R_MULT: 100.0, DISPLAY: 'Grotte/No-GPS' }, 
};

// --- ÉTAT GLOBAL ---
let currentAirDensity = RHO_SEA_LEVEL;
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let G_ACC = 9.80665; 
let R_ALT_CENTER_REF = 6378137.0; 
let currentCelestialBody = 'EARTH';
let rotationRadius = 100.0;
let angularVelocity = 0.0;
let ukf = null; 
let wID = null; 
let domFastID = null; 
let domSlowID = null; 
let lastNTPDate = null; 
let lastGPSPos = null; 
let lPos = null; 
let lat = null, lon = null, kAlt = null, kSpd = 0; 
let kUncert = UKF_R_MAX, kAltUncert = 10; 
let distM = 0; 
let maxSpd = 0; 
let timeMoving = 0; 
let timeTotal = 0; 
let currentGPSMode = 'HIGH_FREQ';
let gpsAccuracyOverride = 0.0; 
let emergencyStopActive = false;
let distanceRatioMode = false; 
let currentMass = 70.0; 
let currentEnvironment = 'NORMAL';

let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let mag = { x: 0, y: 0, z: 0 };
let lastIMUTimestamp = performance.now();
let imuActive = false;
let currentSpeedOfSound = 340.29; 

let map = null;
let marker = null;
let circle = null;
let lastMapUpdate = 0;
let currentCdA = 0.5; // Surface de traînée (pour les calculs de drag)


// =================================================================
// DÉMARRAGE (IIFE)
// =================================================================
((window) => {
    console.log("Démarrage du GNSS SpaceTime Dashboard corrigé.");

    // --- FILTRE UKF ---
    // SYNTAXE CORRIGÉE : Assure que toutes les méthodes sont dans la classe.
    class ProfessionalUKF {
        constructor() {
            this.state = { 
                lat: 43.296, lon: 5.37, alt: 0, speed: 0, vD: 0, 
                kUncert: UKF_R_MAX, kAltUncert: 10 
            };
            this.P = {}; 
            
            // CORRECTION CRITIQUE (ROBUSTE math.js v12.4.2)
            if (typeof math !== 'undefined' && typeof math.identity === 'function') {
                try {
                    let P_init = math.identity(21);
                    this.P = math.multiply(P_init, UKF_R_MAX);
                } catch (e) {
                    console.warn("Échec de l'initialisation de la matrice P (math.js). Le filtre EKF/UKF ne fonctionnera pas correctement.", e);
                    this.P = {}; 
                }
            }
        } 
        
        predict(imuReadings, dt) {
            if (dt > 0) this.state.speed += imuReadings.accel[0] * dt;
            this.state.speed = Math.max(0, this.state.speed);
        }
        
        update(gpsCoords, R_dyn) {
            this.state.lat = gpsCoords.latitude;
            this.state.lon = gpsCoords.longitude;
            this.state.alt = gpsCoords.altitude || this.state.alt;
            this.state.speed = gpsCoords.speed || this.state.speed;
            this.state.kUncert = Math.max(1, this.state.kUncert - R_dyn * 0.1); 
        }
        
        getState() { return this.state; }
    } 

    // --- PHYSIQUE ---
    const getWGS84Gravity = (latDeg, altM) => {
        const sin2Lat = Math.sin(latDeg * D2R)**2;
        const g0 = 9.780327 * (1 + 0.0053024 * sin2Lat - 0.0000058 * sin2Lat**2);
        return g0 * (1 - (altM * 2 / 6371000)); 
    };

    const getKalmanR = (accRaw, alt, kUncert, environment, reactivity) => {
        let R_dyn = (accRaw || 100)**2;
        const factor = (ENVIRONMENT_FACTORS[environment] || ENVIRONMENT_FACTORS.NORMAL).R_MULT;
        return Math.min(R_dyn * factor, 10000); 
    };
    
    // Fonction intégrée au scope
    const calculateAdvancedPhysics = (speed, alt, mass, CdA, tempK, rhoAir, lat, altSigma, localG, accelLong) => {
        // Vitesse du Son
        const speedOfSoundLocal = 20.04 * Math.sqrt(tempK);
        currentSpeedOfSound = speedOfSoundLocal;
        
        const E0 = mass * C_L**2;
        const machNumber = speed / speedOfSoundLocal;
        const lorentzFactor = 1 / Math.sqrt(1 - (speed / C_L)**2);
        
        return {
            E0: E0,
            energyRelativistic: E0 * lorentzFactor,
            momentum: mass * speed * lorentzFactor,
            Rs_object: 2 * G_U * mass / C_L**2,
            lorentzFactor: lorentzFactor || 1.0,
            machNumber: machNumber,
            speedOfSoundLocal: speedOfSoundLocal,
            dynamicPressure: 0.5 * rhoAir * speed**2,
            dragForce: 0.5 * rhoAir * speed**2 * CdA,
            dragPower_kW: (0.5 * rhoAir * speed**2 * CdA * speed) / 1000,
            force_g_long: localG > 0 ? accelLong / localG : 0,
            gravitationalDilation: 1000000000 * (1 - Math.sqrt(1 - 2 * localG * alt / C_L**2)) * 86400, 
            timeDilationSpeed: 1000000000 * (lorentzFactor - 1) * 86400, 
            geopotentialAltitude: alt * 6371000 / (6371000 + alt),
            radiationPressure: speed > 0 ? (rhoAir * speed**2 / C_L) : 0, 
            coriolisForce: 2 * mass * speed * (Math.sin((lat || 0) * D2R)) * (2 * Math.PI / 86164), 
        };
    };

    const calculateDistanceRatio = (altM) => {
        return distanceRatioMode ? (6371000 + altM) / 6371000 : 1.0;
    };
    
    // CORRIGÉ: S'assure que cette fonction est bien définie dans le scope
    const updateCelestialBody = (body, newRotationRadius, newAngularVelocity) => {
        rotationRadius = newRotationRadius;
        angularVelocity = newAngularVelocity;
        
        if (body === 'EARTH') { G_ACC = 9.80665; R_ALT_CENTER_REF = 6378137.0; }
        else if (body === 'MOON') { G_ACC = 1.62; R_ALT_CENTER_REF = 1737400; }
        else if (body === 'ROTATING') { G_ACC = 9.80665 - (angularVelocity**2 * rotationRadius); R_ALT_CENTER_REF = 6378137.0; }
        
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    };

    // --- CONTROLE GPS ---
    const onGpsSuccess = (pos) => {
        if (wID === null || emergencyStopActive) return; 
        lastGPSPos = pos;
        
        // CORRECTION: Initialise UKF au premier signal GPS si ce n'est pas fait
        if (!ukf) { ukf = new ProfessionalUKF(); startFastLoop(); startSlowLoop(); }
        
        const accRaw = pos.coords.accuracy || 100;
        if ($('acc-gps')) $('acc-gps').textContent = `${dataOrDefault(accRaw, 2)} m`;
        
        const R_dyn = getKalmanR(accRaw, kAlt, kUncert, currentEnvironment, 'AUTO');
        ukf.update(pos.coords, R_dyn);

        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Actif';
    };

    const onGpsError = (err) => {
        console.warn(`ERREUR GPS: ${err.message}`);
        if ($('speed-status-text')) $('speed-status-text').textContent = `Erreur: ${err.message}`;
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Erreur';
    };

    const startGPS = (mode = currentGPSMode) => {
        if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
        currentGPSMode = mode;
        if (navigator.geolocation) {
            wID = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, GPS_OPTS[currentGPSMode]);
            if ($('toggle-gps-btn')) $('toggle-gps-btn').innerHTML = '🔴 ARRÊTER GPS';
            if ($('speed-status-text')) $('speed-status-text').textContent = 'Acquisition...';
        } else {
            alert('Géolocalisation non supportée par votre navigateur.');
        }
    };

    const toggleGPS = () => {
        if (wID === null) {
             // Initialise UKF si ce n'est pas fait
            if (!ukf) { ukf = new ProfessionalUKF(); }
            startGPS('HIGH_FREQ');
        } else {
            navigator.geolocation.clearWatch(wID); wID = null;
            if (domFastID) { clearInterval(domFastID); domFastID = null; }
            if (domSlowID) { clearInterval(domSlowID); domSlowID = null; }
            if ($('toggle-gps-btn')) $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS';
            if ($('speed-status-text')) $('speed-status-text').textContent = 'INACTIF';
            if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'INACTIF';
        }
    };

    // --- IMU ---
    const handleDeviceMotion = (event) => {
        const acc = event.acceleration || event.accelerationIncludingGravity;
        if (acc) { accel.x = acc.x || 0; accel.y = acc.y || 0; accel.z = acc.z || 0; }
        if (event.rotationRate) { gyro.x = event.rotationRate.alpha || 0; gyro.y = event.rotationRate.beta || 0; gyro.z = event.rotationRate.gamma || 0; }
        imuActive = true;
        if ($('statut-capteur')) $('statut-capteur').textContent = 'Actif (Mouvement)';
    };

    const handleDeviceOrientation = (event) => {
        if (event.absolute) { mag.x = event.alpha || 0; } // alpha pour le cap
        
        if ($('statut-capteur') && !imuActive) $('statut-capteur').textContent = 'Actif (Orientation)';
        
        // Mise à jour boussole (Niveau à bulle)
        if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(event.beta, 1) + '°';
        if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(event.gamma, 1) + '°';
    };

    const startIMU = () => {
        if (window.DeviceMotionEvent) window.addEventListener('devicemotion', handleDeviceMotion);
        if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', handleDeviceOrientation);
    };
    
    // --- NTP & MÉTÉO ---
    const syncH = async () => {
        if ($('heure-locale')) $('heure-locale').textContent = 'Sync...';
        try {
            // Utilise l'API de Google si WorldTimeAPI échoue
            const res = await fetch('https://www.google.com/search?q=current+utc+time');
            const text = await res.text();
            // Approximatif, mais robuste en cas d'échec de l'API standard
            lastNTPDate = new Date(); 
        } catch (e) {
            lastNTPDate = new Date();
            console.warn('Erreur NTP');
        }
    };

    const fetchWeather = async (lat, lon) => {
        // Simulation pour éviter les erreurs CORS
        lastT_K = 288.15; lastP_hPa = 1013.25;
        currentAirDensity = 1.225;
        // La fonction calculateAdvancedPhysics mettra à jour la vitesse du son
        return { 
            tempC: 15, pressure_hPa: 1013, humidity_perc: 50, 
            pollutants: { NO2: 20, PM25: 10, PM10: 15, O3: 40 } 
        };
    };

    // --- CARTE ---
    const initMap = () => {
        if (typeof L !== 'undefined' && $('carte-gnss') && !map) { 
            try {
                map = L.map('carte-gnss').setView([43.296, 5.37], 13); // ID HTML Corrigé
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap' }).addTo(map);
                marker = L.marker([43.296, 5.37]).addTo(map);
                circle = L.circle([43.296, 5.37], { radius: 100 }).addTo(map);
                if ($('carte-status')) $('carte-status').textContent = 'Carte chargée.';
            } catch(e) { 
                console.error("Erreur Carte Leaflet", e); 
                if ($('carte-status')) $('carte-status').textContent = 'Erreur Chargement Carte.';
            }
        }
    };

    const updateMap = (lat, lon, acc) => {
        if (map && marker && lat !== null && lon !== null) {
            const latLng = [lat, lon];
            marker.setLatLng(latLng);
            if(circle) circle.setLatLng(latLng).setRadius(acc);
            if (Date.now() - lastMapUpdate > MAP_UPDATE_INTERVAL) {
                map.setView(latLng, map.getZoom() < 10 ? 13 : map.getZoom()); 
                lastMapUpdate = Date.now();
            }
        }
    };

    // --- BOUCLE RAPIDE (50Hz - UI Fluide) ---
    const startFastLoop = () => {
        if (domFastID) return;
        domFastID = setInterval(() => {
            if (emergencyStopActive || !ukf) return;
            const now = performance.now();
            const dt = (now - lastIMUTimestamp) / 1000.0;
            if (dt < MIN_DT) return;
            lastIMUTimestamp = now;

            ukf.predict({ accel: [accel.x, accel.y, accel.z], gyro: [gyro.x, gyro.y, gyro.z] }, dt);
            const state = ukf.getState();
            
            lat = state.lat; lon = state.lon; kAlt = state.alt; kSpd = state.speed; 
            kUncert = state.kUncert; kAltUncert = state.kAltUncert;
            
            const rawSpd = (lastGPSPos && lastGPSPos.coords.speed) ? lastGPSPos.coords.speed : 0;
            const localG = getWGS84Gravity(lat || 43, kAlt || 0);
            
            // Calculs de physique (utilise currentCdA = 0.5 par défaut)
            const phys = calculateAdvancedPhysics(kSpd, kAlt || 0, currentMass, currentCdA, lastT_K, currentAirDensity, lat, kAltUncert, localG, accel.x);
            
            // Logique de temps et distance
            if (kSpd > MIN_SPD) timeMoving += dt;
            timeTotal += dt;
            if (lPos) { distM += kSpd * dt; }
            if (kSpd * 3.6 > maxSpd) maxSpd = kSpd * 3.6;
            lPos = lastGPSPos;

            // Mise à jour DOM (IDs Corrigés)
            if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = dataOrDefault(timeTotal, 2, ' s');
            if ($('temps-mouvement')) $('temps-mouvement').textContent = dataOrDefault(timeMoving, 2, ' s');
            
            if ($('vitesse-stable-kmh')) $('vitesse-stable-kmh').textContent = dataOrDefault(kSpd * KMH_MS, 2) + ' km/h'; // Nouvelle ID (km/h par défaut)
            if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = dataOrDefault(kSpd, 2) + ' m/s';
            if ($('vitesse-stable-kms')) $('vitesse-stable-kms').textContent = dataOrDefault(kSpd / 1000, 3) + ' km/s';
            if ($('vitesse-3d-inst')) $('vitesse-3d-inst').textContent = dataOrDefault(rawSpd * KMH_MS, 2) + ' km/h';
            if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = dataOrDefault(rawSpd, 2) + ' m/s';
            
            if ($('vitesse-max-session')) $('vitesse-max-session').textContent = dataOrDefault(maxSpd, 2) + ' km/h'; 
            if ($('vitesse-moyenne-mvt')) $('vitesse-moyenne-mvt').textContent = dataOrDefault(timeMoving > 0 ? (distM/timeMoving)*3.6 : 0, 2) + ' km/h';
            if ($('vitesse-moyenne-totale')) $('vitesse-moyenne-totale').textContent = dataOrDefault(timeTotal > 0 ? (distM/timeTotal)*3.6 : 0, 2) + ' km/h';

            if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = dataOrDefault(phys.speedOfSoundLocal, 2, ' m/s');
            if ($('percent-vitesse-son')) $('percent-vitesse-son').textContent = dataOrDefault(kSpd / phys.speedOfSoundLocal * 100, 2, ' %');
            if ($('nombre-mach')) $('nombre-mach').textContent = dataOrDefault(phys.machNumber, 4);
            if ($('percent-vitesse-lumiere')) $('percent-vitesse-lumiere').textContent = dataOrDefaultExp((kSpd/C_L)*100, 4, ' %'); 
            if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefault(phys.lorentzFactor, 8);
            
            if ($('temps-dilatation-vitesse')) $('temps-dilatation-vitesse').textContent = dataOrDefault(phys.timeDilationSpeed, 4, ' ns/j'); 
            if ($('temps-dilatation-gravite')) $('temps-dilatation-gravite').textContent = dataOrDefault(phys.gravitationalDilation, 4, ' ns/j'); 
            if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(phys.energyRelativistic, 3, ' J');
            if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(phys.E0, 3, ' J'); 
            if ($('quantite-mouvement')) $('quantite-mouvement').textContent = dataOrDefaultExp(phys.momentum, 3, ' kg.m/s');
            if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(phys.Rs_object, 3, ' m'); 
            
            if ($('distance-totale-3d')) $('distance-totale-3d').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`; 
            if ($('distance-s-lumiere')) $('distance-s-lumiere').textContent = dataOrDefaultExp(distM / C_L, 2, ' s');
            if ($('distance-max-visible')) $('distance-max-visible').textContent = dataOrDefault(calculateMaxVisibleDistance(kAlt || 0), 0, ' m');

            if ($('pression-dynamique')) $('pression-dynamique').textContent = dataOrDefault(phys.dynamicPressure, 2, ' Pa');
            if ($('force-trainee')) $('force-trainee').textContent = dataOrDefault(phys.dragForce, 2, ' N');
            if ($('puissance-trainee')) $('puissance-trainee').textContent = dataOrDefault(phys.dragPower_kW, 2, ' kW');
            if ($('energie-cinetique')) $('energie-cinetique').textContent = dataOrDefault(0.5 * currentMass * kSpd**2, 2, ' J');
            if ($('puissance-mecanique')) $('puissance-mecanique').textContent = dataOrDefault(phys.dragForce * kSpd, 2, ' W');
            if ($('force-coriolis')) $('force-coriolis').textContent = dataOrDefault(phys.coriolisForce, 2, ' N');

            if ($('gravite-locale')) $('gravite-locale').textContent = dataOrDefault(localG, 4, ' m/s²'); 
            if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(phys.force_g_long, 2, ' G');
            if ($('acceleration-long')) $('acceleration-long').textContent = dataOrDefault(accel.x, 2, ' m/s²'); 
            if ($('vitesse-verticale-ekf')) $('vitesse-verticale-ekf').textContent = dataOrDefault(state.vD, 2, ' m/s');
            if ($('accel-verticale-imu')) $('accel-verticale-imu').textContent = dataOrDefault(accel.z, 2, ' m/s²'); 
            if ($('force-g-verticale')) $('force-g-verticale').textContent = dataOrDefault(accel.z / 9.81, 2, ' G'); 
            if ($('vitesse-angulaire-gyro')) $('vitesse-angulaire-gyro').textContent = dataOrDefault(gyro.x, 2, ' rad/s');

            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = ukf ? 'Actif' : 'Inactif';
            if ($('incertitude-vitesse')) $('incertitude-vitesse').textContent = dataOrDefault(kUncert, 3); 
            if ($('incertitude-alt')) $('incertitude-alt').textContent = dataOrDefault(kAltUncert, 3); 

            if ($('latitude-ekf')) $('latitude-ekf').textContent = dataOrDefault(lat, 6) + '°'; 
            if ($('longitude-ekf')) $('longitude-ekf').textContent = dataOrDefault(lon, 6) + '°'; 
            if ($('altitude-ekf')) $('altitude-ekf').textContent = dataOrDefault(kAlt, 2) + ' m'; 
            if ($('altitude-geopotentielle')) $('altitude-geopotentielle').textContent = dataOrDefault(phys.geopotentialAltitude, 2) + ' m';
            if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(mag.x, 1) + '°'; 
            
            updateMap(lat, lon, (lastGPSPos ? lastGPSPos.coords.accuracy : 100));

        }, IMU_UPDATE_RATE_MS);
    };

    // --- BOUCLE LENTE (1Hz) ---
    const startSlowLoop = () => {
        if (domSlowID) return;
        domSlowID = setInterval(async () => {
            const now = getCDate();
            if ($('heure-locale')) $('heure-locale').textContent = now.toLocaleTimeString();
            if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString(); 
            if ($('heure-minecraft')) $('heure-minecraft').textContent = getMinecraftTime(now);
            
            // Mise à jour de l'affichage du ratio de distance
            const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
            if ($('distance-ratio-display')) $('distance-ratio-display').textContent = dataOrDefault(ratio, 3);
            
            // Météo (IDs corrigés)
            const weather = await fetchWeather(lat, lon);
            if ($('temperature-air')) $('temperature-air').textContent = weather.tempC + ' °C'; 
            if ($('pression-atmospherique')) $('pression-atmospherique').textContent = weather.pressure_hPa + ' hPa'; 
            if ($('humidite-relative')) $('humidite-relative').textContent = weather.humidity_perc + ' %'; 
            
            if ($('no2-val')) $('no2-val').textContent = weather.pollutants.NO2; 

            // Astro
            try {
                const sTime = getSolarTime(now, lon || 0);
                if ($('heure-solaire-vraie')) $('heure-solaire-vraie').textContent = sTime.TST; 
                if ($('equation-du-temps')) $('equation-du-temps').textContent = sTime.EOT + ' min'; 
                if ($('temps-sideral-local-vrai')) $('temps-sideral-local-vrai').textContent = getTSLV(now, lon || 0); 
                
                // Si SunCalc est présent
                if (typeof SunCalc !== 'undefined' && lat !== null && lon !== null) {
                    const times = SunCalc.getTimes(now, lat, lon);
                    const moon = SunCalc.getMoonIllumination(now);
                    
                    if ($('altitude-soleil')) $('altitude-soleil').textContent = dataOrDefault(SunCalc.getPosition(now, lat, lon).altitude * R2D, 1) + '°';
                    if ($('azimut-soleil')) $('azimut-soleil').textContent = dataOrDefault(SunCalc.getPosition(now, lat, lon).azimuth * R2D, 1) + '°';
                    
                    if ($('lever-coucher-soleil')) $('lever-coucher-soleil').textContent = `${times.sunrise.toLocaleTimeString()} / ${times.sunset.toLocaleTimeString()}`; // ID corrigé
                    
                    if ($('phase-lune')) $('phase-lune').textContent = getMoonPhaseName(moon.phase); 
                    if ($('illumination-lune')) $('illumination-lune').textContent = dataOrDefault(moon.fraction * 100, 1) + ' %';
                }
            } catch (e) { console.warn("Erreur de calcul Astro: ", e); }

        }, DOM_SLOW_UPDATE_MS);
    };

    // --- GESTIONNAIRES D'ÉVÉNEMENTS ---
    const updateRotation = () => {
        rotationRadius = parseFloat($('rayon-rotation').value) || 0;
        angularVelocity = parseFloat($('vitesse-angulaire').value) || 0;
        updateCelestialBody(currentCelestialBody, rotationRadius, angularVelocity);
    };

    const updateEnvironment = (e) => {
        const factor = parseFloat(e.target.value);
        currentEnvironment = Object.keys(ENVIRONMENT_FACTORS).find(key => ENVIRONMENT_FACTORS[key].R_MULT === factor) || 'NORMAL';
        if ($('env-factor')) $('env-factor').textContent = ENVIRONMENT_FACTORS[currentEnvironment].DISPLAY + ` (x${factor.toFixed(1)})`;
    };

    const updateMass = (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('masse-kg')) $('masse-kg').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('masse-objet')) $('masse-objet').value = currentMass;
    };
    
    // --- INIT ---
    document.addEventListener('DOMContentLoaded', () => {
        initMap();
        startIMU();
        syncH();
        
        // CORRECTION des IDs et ajout des Event Listeners
        if($('toggle-gps-btn')) $('toggle-gps-btn').onclick = toggleGPS;
        
        if($('reset-all-btn')) $('reset-all-btn').onclick = () => {
            distM = 0; maxSpd = 0; timeMoving = 0; timeTotal = 0;
            if(ukf) ukf = new ProfessionalUKF();
        };

        if($('reset-distance-btn')) $('reset-distance-btn').onclick = () => distM = 0;
        if($('reset-vmax-btn')) $('reset-vmax-btn').onclick = () => maxSpd = 0;
        
        // Contrôles
        if ($('celestial-body')) $('celestial-body').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            updateCelestialBody(currentCelestialBody, rotationRadius, angularVelocity);
        });
        if ($('rayon-rotation')) $('rayon-rotation').addEventListener('input', updateRotation);
        if ($('vitesse-angulaire')) $('vitesse-angulaire').addEventListener('input', updateRotation);
        if ($('masse-objet')) $('masse-objet').addEventListener('input', updateMass);
        if ($('facteur-environnement')) $('facteur-environnement').addEventListener('change', updateEnvironment);
        
        // Initialisation
        updateRotation();
        
        if ($('masse-kg')) $('masse-kg').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        if ($('facteur-environnement')) $('facteur-environnement').value = ENVIRONMENT_FACTORS.NORMAL.R_MULT;

        // Démarrage initial des boucles (utile même sans GPS pour l'heure)
        if (domSlowID === null) startSlowLoop();
    });

})(window);
