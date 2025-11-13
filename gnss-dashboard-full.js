// =================================================================
// BLOC 1/4 : gnss-dashboard-1-core-math.js
// Constantes, État Global, Utilitaires & EKF 21-États (Core Math)
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
const OMEGA_EARTH = 7.292115e-5; // Vitesse de rotation de la Terre (rad/s)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const GLOBE_SCALE = 1000000;    // Facteur d'échelle pour le rendu 3D
const IMU_FREQUENCY_HZ = 100;
const DT_IMU = 1 / IMU_FREQUENCY_HZ; 
const MIN_SPD_DR = 0.05;        // Vitesse minimale pour le Dead Reckoning (m/s)

// --- DONNÉES PHYSIQUES / ENVIRONNEMENT ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.8067, R_E: 6371000 },
    'MOON': { G: 1.625, R_E: 1737400 },
};

const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, BIOME_COLOR: 0x38761d, DRAG_C: 0.5 }, 
    'FOREST': { R_MULT: 2.0, BIOME_COLOR: 0x1f7400, DRAG_C: 1.0 },
    'CONCRETE': { R_MULT: 5.0, BIOME_COLOR: 0x4a4a4a, DRAG_C: 0.1 },
    'METAL': { R_MULT: 10.0, BIOME_COLOR: 0x7a7a7a, DRAG_C: 0.2 },
};

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, lPos = null; 
let imuIntervalID = null; 
let ekf6dof = null; // Instance de l'EKF
let lastIMUTime = performance.now();
let GRAVITY_BASE = CELESTIAL_DATA.EARTH.G;
let R_LOCAL_BASE = CELESTIAL_DATA.EARTH.R_E;
let selectedEnvironment = 'NORMAL';
let totalDistance3D = 0; 
let timeMoving = 0;
let totalSessionTime = 0.01; 
let max3DSpeed = 0;
let currentMass = 70.0; 
let lastWeatherData = { tempC: 17.3, tempK: 290.45, pressure_hPa: 1021.0, air_density: 1.225, humidity_perc: 81 };
let lastAccelLong = 0;
let sunAltitudeRad = 0;
const $ = id => document.getElementById(id); 

// --- FONCTIONS MATHÉMATIQUES ET PHYSIQUES ---

function calculateSpeedOfSound(tempK) {
    // Vitesse du son c = sqrt(gamma * R_specific * T)
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

function calculateAirDensity(p_Pa, t_K, h_perc = 0.8) {
    // Équation d'état de l'air humide (simplifiée)
    const R_V = 461.5; // Constante spécifique de la vapeur d'eau
    const MW_DRY = 28.9647; // Masse molaire air sec
    const MW_WATER = 18.01528; // Masse molaire eau
    
    const p_v = h_perc * 611.2 * Math.exp((17.67 * (t_K - 273.15)) / (t_K - 29.65)); // Pression de vapeur saturante (approximation)
    const p_d = p_Pa - p_v; // Pression air sec
    
    // Densité rho = (p_d / (R_SPECIFIC_AIR * T_K)) + (p_v / (R_V * T_K))
    return (p_d / (R_SPECIFIC_AIR * t_K)) + (p_v / (R_V * t_K));
}

function calculateLocalGravity(lat, alt) {
    // Formule d'accélération gravitationnelle
    const latRad = lat * D2R;
    // Compensation pour la rotation de la Terre (centrifuge)
    const g_lat = GRAVITY_BASE * (1 + 0.005302 * Math.sin(latRad)**2 - 0.000007 * Math.sin(2 * latRad)**2);
    // Compensation pour l'altitude
    const g_local = g_lat * (1 - 2 * alt / R_LOCAL_BASE);
    return g_local;
}

// --- CLASSE EKF/INS 21DOF ---
class EKF_INS_21_States {
    constructor() {
        this.P = 1000; // Incertitude de covariance (Vitesse)
        this.Q_VEL = 0.1; // Bruit de processus (IMU)
        this.R_ALT = 5.0; // Bruit de mesure altitude
        this.kAltUncert = 100; // Incertitude d'altitude initiale

        this.lat = 0;
        this.lon = 0;
        this.alt = 0; 
        this.speedMS = 0; 
        this.vX = 0; this.vY = 0; this.vZ = 0; // Vitesses Nord/Est/Up (NEU Frame)
        
        // Attitude estimée (Euler)
        this.roll = 0.0; 
        this.pitch = 0.0; 
        this.yaw = 0.0; // Cap en radians
        this.biasGZ = 0; 
        this.deadReckoningError = 0;
    }

    /**
     * Propagation de l'état (Prédiction) basée sur l'IMU (Accélération 6DOF).
     */
    predict(dt, accel, gyroZ) {
        if (dt <= 0) return;
        
        // Mise à jour de l'orientation (Yaw seulement)
        this.yaw = (this.yaw + (gyroZ - this.biasGZ) * dt * D2R + 2 * Math.PI) % (2 * Math.PI); 

        // 2. Rotation de l'Accélération (Body -> Navigation Frame: NEU)
        const phi = this.roll * D2R, theta = this.pitch * D2R, psi = this.yaw;
        const cP = Math.cos(phi), sP = Math.sin(phi);
        const cT = Math.cos(theta), sT = Math.sin(theta);
        const cY = Math.cos(psi), sY = Math.sin(psi);

        // Matrice de Rotation R_b^n (ZYX convention)
        const R11 = cY * cT, R12 = cY * sT * sP - sY * cP, R13 = cY * sT * cP + sY * sP;
        const R21 = sY * cT, R22 = sY * sT * sP + cY * cP, R23 = sY * sT * cP - cY * sP;
        const R31 = -sT, R32 = cT * sP, R33 = cT * cP;
        
        // Accélération dans le référentiel de navigation (NEU)
        const acc_n_x = R11 * accel.x + R12 * accel.y + R13 * accel.z; // North (vX)
        const acc_n_y = R21 * accel.x + R22 * accel.y + R23 * accel.z; // East (vY)
        const acc_n_z = R31 * accel.x + R32 * accel.y + R33 * accel.z; // Up (vZ)
        
        // 3. Intégration de la Vitesse (V = V + (a^n - g^n) * dt)
        const localG = calculateLocalGravity(this.lat, this.alt);
        this.vX += acc_n_x * dt;
        this.vY += acc_n_y * dt;
        this.vZ += (acc_n_z - localG) * dt; // Soustraction de la gravité
        
        // 4. Mise à jour de la Position (Dead Reckoning)
        if (this.lat !== null) {
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
    }
    
    /**
     * Correction de l'état par les mesures GNSS et Barométriques.
     */
    correctGNSS(pos, altBaro) {
        if (pos === null) return;
        
        const accGPS = pos.coords.accuracy || 10.0;
        const gnssSpd3D = pos.coords.speed || 0; 
        
        // Calcul du Bruit de Mesure (R_dyn) : dépend de la précision GPS et de l'environnement
        const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
        const R_dyn = Math.max(1.0, accGPS * envFactor); // Minimum 1.0 m/s d'incertitude
        
        // CORRECTION VITESSE 3D
        const K_vel = this.P / (this.P + R_dyn);
        const error_v = gnssSpd3D - this.speedMS;
        
        if (this.speedMS > 0.01) {
            this.vX += K_vel * error_v * (this.vX / this.speedMS);
            this.vY += K_vel * error_v * (this.vY / this.speedMS);
            this.vZ += K_vel * error_v * (this.vZ / this.speedMS);
        } else {
            // Ralentissement progressif si à l'arrêt
            this.vX = this.vX * (1 - K_vel);
            this.vY = this.vY * (1 - K_vel);
            this.vZ = this.vZ * (1 - K_vel);
        }
        
        this.speedMS = Math.sqrt(this.vX**2 + this.vY**2 + this.vZ**2);
        this.P = (1 - K_vel) * this.P;
        
        // CORRECTION POSITION & ALTITUDE
        const K_pos = 0.5; // Gain simple pour la position
        if (this.lat !== 0) {
            this.lat += K_pos * (pos.coords.latitude - this.lat);
            this.lon += K_pos * (pos.coords.longitude - this.lon);
            this.deadReckoningError = this.deadReckoningError * (1 - K_pos); 

            // Correction Altitude (priorité Baro si disponible)
            const alt_ref = altBaro !== null ? altBaro : pos.coords.altitude;
            if (alt_ref !== null) {
                const K_alt = this.kAltUncert / (this.kAltUncert + this.R_ALT);
                this.alt += K_alt * (alt_ref - this.alt);
                this.kAltUncert = (1 - K_alt) * this.kAltUncert;
            }
        } else {
            // Initialisation
            this.lat = pos.coords.latitude;
            this.lon = pos.coords.longitude;
            this.alt = altBaro || pos.coords.altitude || 0;
            this.yaw = pos.coords.heading !== null ? pos.coords.heading * D2R : 0;
        }

        this.biasGZ *= 0.99; // Dérive du Gyro Z tend vers zéro
    }
}
// Fin BLOC 1/4
// =================================================================
// BLOC 2/4 : gnss-dashboard-2-3d-globe.js
// Fonctions de Rendu 3D (Globe, Trajectoire, Aurores) & Astro
// =================================================================

// --- VARIABLES GLOBALES THREE.JS ---
let scene, camera, renderer, globeMesh, sunLight, userMarker, trajectoryLine, velocityVector, targetMarker;
let trajectoryPoints = [];
let auroraParticles = []; // Pour la simulation des aurores

// --- FONCTIONS DE GÉOMÉTRIE 3D ---

/**
 * Convertit Lat/Lon/Alt en coordonnées 3D (x, y, z) pour Three.js.
 */
function LLAToXYZ(lat, lon, alt) {
    const radius = R_LOCAL_BASE / GLOBE_SCALE + alt / GLOBE_SCALE; 
    const phi = (90 - lat) * D2R; 
    const theta = (lon + 180) * D2R; 

    const x = -radius * Math.sin(phi) * Math.cos(theta);
    const z = radius * Math.sin(phi) * Math.sin(theta);
    const y = radius * Math.cos(phi); 
    
    return new THREE.Vector3(x, y, z);
}

// --- INITIALISATION DU GLOBE ---

function initGlobe3D() {
    const container = $('globe-3d-container');
    if (!container) return;

    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.01, 100);
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.innerHTML = ''; 
    container.appendChild(renderer.domElement);

    // 1. Globe (Base)
    const globeRadius = R_LOCAL_BASE / GLOBE_SCALE;
    const geometry = new THREE.SphereGeometry(globeRadius, 64, 64);
    const texture = new THREE.TextureLoader().load('https://cdn.jsdelivr.net/gh/mrdoob/three.js@r128/examples/textures/planets/earth_atmos_4096.jpg'); 
    const material = new THREE.MeshPhongMaterial({ map: texture, specular: 0x333333, shininess: 15 });
    globeMesh = new THREE.Mesh(geometry, material);
    scene.add(globeMesh);

    // 2. Lumières (Jour/Nuit)
    const ambientLight = new THREE.AmbientLight(0xaaaaaa, 0.5); 
    ambientLight.name = 'ambientLight';
    scene.add(ambientLight);
    sunLight = new THREE.DirectionalLight(0xffffff, 3.0);
    sunLight.name = 'sunLight';
    scene.add(sunLight);
    
    // 3. Objets 3D Dynamiques
    userMarker = new THREE.Mesh(new THREE.SphereGeometry(globeRadius * 0.01, 16, 16), new THREE.MeshBasicMaterial({ color: 0x00ff00 }));
    scene.add(userMarker);
    
    trajectoryLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 2 }));
    scene.add(trajectoryLine);
    
    velocityVector = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.5, 0x00ff00, 0.2, 0.1);
    scene.add(velocityVector);
    
    targetMarker = new THREE.Mesh(new THREE.SphereGeometry(globeRadius * 0.015, 16, 16), new THREE.MeshBasicMaterial({ color: 0xffff00 }));
    targetMarker.visible = false;
    scene.add(targetMarker);
    
    // Contrôles de la caméra
    // const controls = new THREE.OrbitControls(camera, renderer.domElement); // Nécessaire pour interactivité
    
    camera.position.set(0, 0, globeRadius * 2.5);

    function animate() {
        requestAnimationFrame(animate);
        globeMesh.rotation.y += 0.0005; // Rotation pour le dynamisme
        renderer.render(scene, camera);
    }
    animate();
}

// --- MISE À JOUR DYNAMIQUE ---

function updateGlobe3D() {
    if (!ekf6dof || !globeMesh || !sunLight) return;
    
    const { lat, lon, alt, vX, vY, vZ } = ekf6dof;
    const currentPos3D = LLAToXYZ(lat, lon, alt);
    const globeRadius = R_LOCAL_BASE / GLOBE_SCALE;
    
    // 1. Position Utilisateur
    userMarker.position.copy(currentPos3D);
    
    // 2. Trajectoire 3D
    trajectoryPoints.push(currentPos3D);
    if (trajectoryPoints.length > 500) trajectoryPoints.shift(); 
    trajectoryLine.geometry.setFromPoints(trajectoryPoints);
    trajectoryLine.geometry.attributes.position.needsUpdate = true;
    
    // 3. Vecteur Vitesse
    const velocity = new THREE.Vector3(vX, vY, vZ);
    const direction = velocity.clone().normalize();
    const length = ekf6dof.speedMS / KMH_MS * 0.01; 
    
    velocityVector.position.copy(currentPos3D);
    velocityVector.setDirection(direction);
    velocityVector.setLength(length, 0.2 * length, 0.1 * length);
    
    // 4. Modélisation Souterraine (Écholocalisation)
    const biomeColor = ENVIRONMENT_FACTORS[selectedEnvironment].BIOME_COLOR;
    userMarker.material.color.setHex(ekf6dof.speedMS > MIN_SPD_DR ? 0x00ff00 : 0x00aaff);

    if (alt < -10) { 
        globeMesh.material.opacity = 0.5;
        globeMesh.material.transparent = true;
        // La trajectoire et le marqueur sont maintenant "sous" la surface
        userMarker.material.color.setHex(biomeColor); 
    } else {
        globeMesh.material.opacity = 1.0;
        globeMesh.material.transparent = false;
    }
    
    // 5. Cycle Jour/Nuit (Mise à jour de la lumière solaire)
    if (ekf6dof.lat !== 0) {
        // Obtenir la position du soleil (nécessite suncalc)
        const sunPos = SunCalc.getPosition(new Date(), lat, lon);
        sunAltitudeRad = sunPos.altitude;
        const sunAzimuth = sunPos.azimuth;

        // Convertir Azimuth/Altitude en position 3D de la lumière solaire
        const sunLightPos = LLAToXYZ(sunPos.altitude * R2D, sunPos.azimuth * R2D, globeRadius * 5); // Position loin
        sunLight.position.copy(sunLightPos);
        sunLight.target.position.copy(currentPos3D); // Pointer vers l'utilisateur
        
        // Simuler le cycle jour/nuit par l'intensité lumineuse
        const intensity = Math.max(0.1, Math.min(1.0, sunAltitudeRad / (30 * D2R))); // Crépuscule/Jour
        sunLight.intensity = 3.0 * intensity;
    }
    
    // 6. Affichage du Biome
    document.getElementById('current-biome').textContent = detectBiome(lat, alt); 
}

/**
 * Définit la cible GPS sur le globe 3D.
 */
function setTargetCoordinates() {
    const targetLat = parseFloat($('target-lat').value);
    const targetLon = parseFloat($('target-lon').value);
    
    if (isNaN(targetLat) || isNaN(targetLon)) return;

    const targetAlt = 0; 
    const targetPos3D = LLAToXYZ(targetLat, targetLon, targetAlt);
    targetMarker.position.copy(targetPos3D);
    targetMarker.visible = true;
}
// Fin BLOC 2/4
// =================================================================
// BLOC 3/4 : gnss-dashboard-3-sensor-loops.js
// Fonctions de Simulation Capteurs & Logique de Commande (EKF/IMU Loop)
// =================================================================

let lServH = new Date(), lLocH = new Date(); // Temps serveur et temps local

// --- UTILS HORLOGE NTP ---

/** Calcule l'heure corrigée après synchronisation NTP. */
function getCDate(servTime, localTime) {
    if (!servTime || !localTime) return null;
    const delta = servTime.getTime() - localTime.getTime();
    return new Date(Date.now() + delta);
}

/** Synchronise le temps avec un serveur externe. */
function syncH() {
    fetch(SERVER_TIME_ENDPOINT)
        .then(response => response.json())
        .then(data => {
            lServH = new Date(data.utc_datetime);
            lLocH = new Date();
        })
        .catch(err => {
            console.error("Erreur de synchronisation NTP:", err);
            if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE (Local)';
        });
}

// --- GESTION DE LA FUSION DE CAPTEURS ---

function startFusion() {
    if (wID !== null) return;
    
    ekf6dof = new EKF_INS_21_States(); 
    initGlobe3D(); 
    syncH(); // Synchronisation initiale

    // 1. Boucle IMU (Haute Fréquence - Prédiction EKF)
    lastIMUTime = performance.now();
    imuIntervalID = setInterval(runIMUIntegration, DT_IMU * 1000);

    // 2. GPS (Basse Fréquence - Correction EKF)
    wID = navigator.geolocation.watchPosition(
        (pos) => { 
            lPos = pos;
            const altBaro = lastWeatherData.baro_alt_msl || pos.coords.altitude; 
            ekf6dof.correctGNSS(lPos, altBaro); 
            
            if (ekf6dof.lat !== 0) fetchWeatherAndSync(ekf6dof.lat, ekf6dof.lon);
        },
        (error) => { console.warn(`GPS ERROR(${error.code}): ${error.message}`); },
        { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
    );
    
    // 3. Boucle Lente (Météo/DOM)
    setInterval(updateDisplaySlow, DOM_SLOW_UPDATE_MS);
    
    $('gps-status-dr').textContent = 'EKF/INS ACTIF';
    $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
}

function stopFusion() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    if (imuIntervalID !== null) clearInterval(imuIntervalID); 
    wID = null;
    imuIntervalID = null;
    ekf6dof = null;
    $('gps-status-dr').textContent = 'Arrêté';
    $('toggle-gps-btn').textContent = "▶️ Démarrer la Fusion (INS/GNSS)";
}

// --- BOUCLE D'INTÉGRATION IMU (Le Cœur du EKF) ---

function runIMUIntegration() {
    if (!ekf6dof) return;

    const cTime = performance.now();
    const dt = (cTime - lastIMUTime) / 1000;
    lastIMUTime = cTime;
    
    if (dt > 0.5) return;
    
    totalSessionTime += dt;
    
    // --- SIMULATION DES DONNÉES BRUTES IMU ---
    const imuAccelZ_sim = 0.1 * Math.sin(cTime / 10000); // Mouvement vertical
    const imuAccelX_sim = 0.2 * Math.cos(cTime / 8000); // Accélération Long.
    lastAccelLong = imuAccelX_sim;
    
    const accel = { 
        x: imuAccelX_sim, 
        y: 0.05 * Math.sin(cTime / 12000), // Latérale
        z: imuAccelZ_sim // Verticale
    };
    const gyroZ = 0.5 * Math.sin(cTime / 15000); // Vitesse angulaire Yaw

    // Simuler l'attitude (à remplacer par une estimation EKF complète)
    ekf6dof.roll = 0.5 * Math.sin(cTime / 8000) * R2D;
    ekf6dof.pitch = 1.0 * Math.cos(cTime / 9000) * R2D;
    
    // EKF PREDICTION
    ekf6dof.predict(dt, accel, gyroZ);
    
    // Mise à jour des compteurs et de la distance 3D
    const distStep = ekf6dof.speedMS * dt;
    totalDistance3D += distStep;
    if (ekf6dof.speedMS > MIN_SPD_DR) timeMoving += dt;
    if (ekf6dof.speedMS > max3DSpeed) max3DSpeed = ekf6dof.speedMS;

    // Mettre à jour l'affichage de l'IMU brute
    $('imu-accel-total').textContent = `${Math.sqrt(accel.x**2 + accel.y**2 + accel.z**2).toFixed(2)} m/s²`;
    $('imu-accel-xyz').textContent = `${accel.x.toFixed(2)} / ${accel.y.toFixed(2)} / ${accel.z.toFixed(2)} m/s²`;
    $('imu-gyro-z').textContent = `${gyroZ.toFixed(2)} °/s`;
    $('bias-gyro-z').textContent = `${ekf6dof.biasGZ.toFixed(3)} °/s`;
    $('imu-status').textContent = 'Actif (Simulé)';

    updateGlobe3D(); 
}

/**
 * Récupère les données météo et synchronise l'heure.
 */
function fetchWeatherAndSync(lat, lon) {
    // 1. Météo
    fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`)
        .then(response => response.json())
        .then(data => {
            if (data && data.tempK) {
                lastWeatherData = {
                    tempC: data.tempC,
                    tempK: data.tempK,
                    pressure_hPa: data.pressure_hPa,
                    humidity_perc: data.humidity_perc,
                    air_density: calculateAirDensity(data.pressure_hPa * 100, data.tempK, data.humidity_perc / 100),
                    baro_alt_msl: data.baro_alt_msl // Altitude MSL par API Météo
                };
            }
        })
        .catch(err => console.error("Erreur API Météo:", err));

    // 2. Synchronisation NTP (relance après la première fois)
    syncH();
}
// Fin BLOC 3/4
// =================================================================
// BLOC 4/4 : gnss-dashboard-4-dom-events.js
// Mises à Jour du DOM (Affichage) & Initialisation
// =================================================================

// --- FONCTIONS SVT & PHYSIQUE POUR L'AFFICHAGE ---

/**
 * Simule la détection de biome (guidée par l'écholocalisation si souterrain).
 */
function detectBiome(lat, alt) {
    if (alt < -100) return "Caverne Abyssale (Magmatique)";
    if (alt < -10) return "Galerie Souterraine (Roche Sédimentaire)";
    
    if (lat > 66 || lat < -66) {
        if (sunAltitudeRad < 0) return "Polaire (Nuit / Aurores)";
        return "Polaire (Jour)";
    }
    
    return "Tempéré / " + selectedEnvironment;
}

/**
 * Calcule les forces dynamiques (Traînée, Coriolis, Pression dynamique).
 */
function calculateForces(v, mass, rho) {
    const C_d = ENVIRONMENT_FACTORS[selectedEnvironment].DRAG_C; // Coefficient de Traînée
    const A = 0.5; // Surface de référence (m²) simulée
    
    // Pression Dynamique: q = 0.5 * rho * v²
    const q = 0.5 * rho * v**2;
    
    // Force de Traînée: F_d = q * C_d * A
    const dragForce = q * C_d * A;
    
    // Force de Coriolis (simplifiée, horizontale)
    const corioForce = 2 * mass * OMEGA_EARTH * v * Math.sin(ekf6dof.lat * D2R);
    
    return { q, dragForce, corioForce };
}

// --- BOUCLE LENTE DE MISE À JOUR DU DOM ---

function updateDisplaySlow() {
    if (!ekf6dof) return;
    
    const now = getCDate(lServH, lLocH);
    const { lat, lon, alt, speedMS, roll, pitch, yaw, P, kAltUncert } = ekf6dof;
    const { tempC, tempK, pressure_hPa, air_density, humidity_perc } = lastWeatherData;

    // 1. EKF & Position
    $('dead-reckoning-error').textContent = `${ekf6dof.deadReckoningError.toFixed(1)} m`;
    $('latitude').textContent = `${lat.toFixed(6)} °`;
    $('longitude').textContent = `${lon.toFixed(6)} °`;
    $('altitude-amsl').textContent = `${alt.toFixed(2)} m`;
    $('heading-display').textContent = `${(yaw * R2D).toFixed(1)} °`;
    $('roll-pitch-display').textContent = `${roll.toFixed(2)}° / ${pitch.toFixed(2)}°`;
    $('kalman-uncert').textContent = `${P.toFixed(3)} m/s`;
    $('alt-uncertainty').textContent = `${Math.sqrt(kAltUncert).toFixed(2)} m`;
    const R_current = ekf6dof.P / ((ekf6dof.P / (pos.coords.accuracy || 10.0)) || 1.0); // Simplification
    $('speed-error-perc').textContent = `${R_current.toFixed(1)} m/s`;

    // 2. Vitesse et Relativité
    $('speed-stable-3d-kmh').textContent = `${(speedMS * KMH_MS).toFixed(1)} km/h`;
    $('speed-stable-3d-ms').textContent = `${speedMS.toFixed(2)} m/s`;
    $('speed-max-3d').textContent = `${(max3DSpeed * KMH_MS).toFixed(1)} km/h`;
    $('distance-total-km').textContent = `${(totalDistance3D / 1000).toFixed(3)} km | ${totalDistance3D.toFixed(1)} m`;
    
    const speedSound = calculateSpeedOfSound(tempK);
    const mach = speedMS / speedSound;
    const perc_c = (speedMS / C_L) * 100;
    const lorentz_factor = 1 / Math.sqrt(1 - (speedMS / C_L)**2);
    
    $('speed-sound-local').textContent = `${speedSound.toFixed(1)} m/s`;
    $('mach-number').textContent = `${mach.toFixed(4)}`;
    $('perc-speed-c').textContent = `${perc_c.toExponential(2)} %`;
    $('lorentz-factor').textContent = `${lorentz_factor.toFixed(4)}`;

    // 3. Dynamique et Forces
    const localG = calculateLocalGravity(lat, alt);
    const { q, dragForce, corioForce } = calculateForces(speedMS, currentMass, air_density);
    const g_force_long = lastAccelLong / localG;

    $('gravity-local').textContent = `${localG.toFixed(4)} m/s²`;
    $('force-g-long').textContent = `${g_force_long.toFixed(2)} G`;
    $('drag-force').textContent = `${dragForce.toFixed(2)} N`;
    $('coriolis-force').textContent = `${corioForce.toFixed(2)} N`;
    $('dynamic-pressure').textContent = `${q.toFixed(2)} Pa`;

    // 4. Météo et Chimie
    $('temp-air-2').textContent = `${tempC.toFixed(1)} °C`;
    $('pressure-2').textContent = `${pressure_hPa.toFixed(0)} hPa`;
    $('humidity-2').textContent = `${humidity_perc} %`;
    $('air-density').textContent = `${air_density.toFixed(3)} kg/m³`;
    // Calcul Point de Rosée (simplifié)
    const dewPoint = tempC - ((100 - humidity_perc) / 5); 
    $('dew-point').textContent = `${dewPoint.toFixed(1)} °C`;
    
    // 5. Astro (utiliser SunCalc.getTimes pour les heures de crépuscule)
    if (now && lat !== 0) {
        const times = SunCalc.getTimes(now, lat, lon);
        const sunPos = SunCalc.getPosition(now, lat, lon);
        
        $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D + 180).toFixed(1)} °`;
        $('sun-altitude').textContent = `${(sunPos.altitude * R2D).toFixed(1)} °`;
        $('twilight-time').textContent = `Nuit: ${times.night.toLocaleTimeString()} - Aube: ${times.nauticalDawn.toLocaleTimeString()}`;
    }

    // 6. Système
    $('time-elapsed').textContent = `${totalSessionTime.toFixed(1)} s`;
    $('time-moving').textContent = `${timeMoving.toFixed(1)} s`;
    if (now) {
        if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    }
}

// --- INITIALISATION DES ÉVÉNEMENTS DOM ---

document.addEventListener('DOMContentLoaded', () => {
    
    // Initialisation
    $('current-biome').textContent = detectBiome(0, 0);
    syncH();
    
    // --- Listeners de Contrôle ---
    $('toggle-gps-btn').addEventListener('click', () => { 
        ekf6dof === null ? startFusion() : stopFusion(); 
    });
    
    $('set-target-btn').addEventListener('click', setTargetCoordinates);

    $('celestial-body-select').addEventListener('change', (e) => {
        const body = CELESTIAL_DATA[e.target.value];
        GRAVITY_BASE = body.G;
        R_LOCAL_BASE = body.R_E;
        $('gravity-base').textContent = `${body.G.toFixed(4)} m/s²`;
    });

    $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
        $('env-factor').textContent = `${selectedEnvironment} (x${factor.R_MULT.toFixed(1)})`;
    });
    
    $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    
    // Lancement de l'affichage lent pour la première fois
    updateDisplaySlow(); 
    // La boucle lente sera lancée dans startFusion
});
// Fin BLOC 4/4
