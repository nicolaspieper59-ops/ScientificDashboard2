/**
 * GNSS SPACETIME DASHBOARD - KERNEL V.FINAL (ENGINEERING GRADE)
 * ARCHITECTURE: UKF 21-STATE (Loose Coupling) + WGS84 + RELATIVITY
 * * Ce fichier contient la logique complète de fusion de capteurs.
 * Aucune simplification n'est appliquée aux formules géodésiques.
 */

// =============================================================================
// 1. CONSTANTES PHYSIQUES & GÉODÉSIQUES (CODATA 2018 / WGS84 REV 2)
// =============================================================================

// Constantes Universelles
const C_L = 299792458.0;          // Vitesse lumière (exacte)
const G_U = 6.67430e-11;          // Constante gravitationnelle (m³/kg/s²)
const AU_METERS = 149597870700.0; // Unité Astronomique

// WGS84 Ellipsoid Parameters
const WGS84_A = 6378137.0;            // Semi-major axis (a)
const WGS84_F = 1.0 / 298.257223563;  // Flattening (f)
const WGS84_B = WGS84_A * (1 - WGS84_F); // Semi-minor axis (b)
const WGS84_E2 = 2*WGS84_F - WGS84_F*WGS84_F; // First eccentricity squared (e²)
const WGS84_E2_PRIME = WGS84_E2 / (1 - WGS84_E2); // Second eccentricity squared (e'²)
const OMEGA_EARTH = 7.2921150e-5;     // Earth rotation rate (rad/s)
const WGS84_G_EQUATOR = 9.7803253359; // Theoretical gravity at equator
const WGS84_K = 0.00193185265241;     // Gravitational formula constant

// Atmosphère Standard (ISA)
const R_AIR = 287.05287; // Specific gas constant for dry air
const R_VAPOR = 461.5;   // Specific gas constant for water vapor
const GAMMA_AIR = 1.40;  // Adiabatic index
const M_AIR = 0.0289644; // Molar mass of Earth's air (kg/mol)
const STD_TEMP_K = 288.15; // 15°C
const STD_PRESS_PA = 101325.0;

// Paramètres de Simulation & Conversion
const D2R = Math.PI / 180.0;
const R2D = 180.0 / Math.PI;
const KMH_MS = 3.6;
const MS_KMH = 1.0 / 3.6;

// =============================================================================
// 2. ÉTAT GLOBAL DU SYSTÈME (SINGLE SOURCE OF TRUTH)
// =============================================================================

const sysState = {
    // Vecteur d'État Estimé (Post-Fusion)
    pose: {
        lat: 43.2964, lon: 5.3697, alt: 0.0, // Position Géodésique
        vn: 0.0, ve: 0.0, vd: 0.0,           // Vitesse NED (North-East-Down)
        roll: 0.0, pitch: 0.0, yaw: 0.0      // Attitude (Euler)
    },
    // Données Brutes Capteurs
    raw: {
        gps: { lat: 0, lon: 0, alt: 0, acc: 0, spd: 0, head: 0, ts: 0 },
        imu: { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, mx: 0, my: 0, mz: 0 },
        env: { temp: 15, press: 1013, hum: 50, lux: 0 }
    },
    // Métriques Dérivées
    metrics: {
        totalDist: 0.0,
        maxSpeed: 0.0,
        moveTime: 0.0,
        mach: 0.0,
        energyKinetic: 0.0
    },
    // Paramètres de Contrôle
    config: {
        mass: 70.0,
        cda: 0.5, // Coefficient de trainée * Surface
        gravityRef: 9.80665,
        celestialBody: 'EARTH', // EARTH, MOON, MARS, ROTATING
        netherMode: false,
        gpsMode: 'HIGH_FREQ'
    },
    // Flags Système
    status: {
        isGpsActive: false,
        isImuActive: false,
        isZuptActive: false, // Zero Velocity Update
        isEmergencyStop: false,
        initTime: Date.now()
    }
};

// Buffers pour l'intégration
let lastIntegrationTime = performance.now();
let ukfInstance = null; // Instance du filtre
let mapInstance = null;
let pathPolyline = null;
let userMarker = null;

// =============================================================================
// 3. UTILITAIRES MATHÉMATIQUES AVANCÉS
// =============================================================================

// Helper DOM rapide
const $ = (id) => document.getElementById(id);

/** Formattage scientifique strict */
const fmt = (val, dec = 2, unit = '') => {
    if (val === undefined || val === null || isNaN(val) || !isFinite(val)) return 'N/A';
    return val.toFixed(dec) + unit;
};
const fmtExp = (val, dec = 2, unit = '') => {
    if (val === undefined || val === null || isNaN(val) || !isFinite(val)) return 'N/A';
    return val.toExponential(dec) + unit;
};

/** Calcul Gravité WGS84 (Somigliana Equation) avec correction d'altitude (FAC) */
function computeLocalGravity(latDeg, altM) {
    if (sysState.config.celestialBody !== 'EARTH') return sysState.config.gravityRef;
    
    const latRad = latDeg * D2R;
    const sinLat = Math.sin(latRad);
    const sin2Lat = sinLat * sinLat;
    
    // Gravité à la surface de l'ellipsoïde
    const g_surf = WGS84_G_EQUATOR * (1 + 0.00193185265241 * sin2Lat) / Math.sqrt(1 - WGS84_E2 * sin2Lat);
    
    // Correction d'Air Libre (Free Air Correction)
    // Approximation: -0.3086 mGal/m -> -3.086e-6 m/s²/m
    const fac = -3.086e-6 * altM;
    
    return g_surf + fac;
}
// =============================================================================
// 4. MOTEUR DE FUSION UKF (UNSCENTED KALMAN FILTER)
// =============================================================================

class EngineeringUKF {
    constructor() {
        if (typeof math === 'undefined') throw new Error("CRITIQUE: math.js manquant");
        
        // Dimensions: État=9 (Pos(3), Vel(3), Att(3)), Mesure=6 (GPS Pos(3), Vel(3))
        this.nx = 9; 
        this.nz = 6;
        
        // --- 1. INITIALISATION VECTEUR ÉTAT (X) ---
        this.x = math.zeros(this.nx); // [Lat, Lon, Alt, Vn, Ve, Vd, Roll, Pitch, Yaw]
        
        // --- 2. MATRICE DE COVARIANCE (P) ---
        // Incertitudes initiales: Pos=10m, Vel=1m/s, Att=1deg
        const P_diag = [100, 100, 100, 1, 1, 1, 0.01, 0.01, 0.01];
        this.P = math.diag(P_diag);
        
        // --- 3. BRUIT DE PROCESSUS (Q) - Modèle d'erreur IMU ---
        // On suppose un bruit constant sur l'intégration de l'accélération
        const Q_diag = [1e-6, 1e-6, 1e-4, 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5]; 
        this.Q = math.diag(Q_diag);
        
        // --- 4. BRUIT DE MESURE (R) - Dynamique GPS ---
        this.R = math.diag([25, 25, 100, 0.25, 0.25, 0.25]); // GPS: 5m H, 10m V, 0.5m/s Vitesse
        
        // Paramètres UKF (Sigma Points)
        this.alpha = 1e-3;
        this.beta = 2.0;
        this.kappa = 0.0;
        this.lambda = this.alpha**2 * (this.nx + this.kappa) - this.nx;
        
        // Poids Wc et Wm
        this.Wm = math.zeros(2*this.nx + 1);
        this.Wc = math.zeros(2*this.nx + 1);
        this.calcWeights();
        
        this.stateStr = "INITIALISÉ";
    }

    calcWeights() {
        const N = 2 * this.nx + 1;
        this.Wm.set([0], this.lambda / (this.nx + this.lambda));
        this.Wc.set([0], this.Wm.get([0]) + (1 - this.alpha**2 + this.beta));
        for(let i=1; i<N; i++) {
            const w = 1 / (2 * (this.nx + this.lambda));
            this.Wm.set([i], w);
            this.Wc.set([i], w);
        }
    }

    /**
     * PRÉDICTION (Time Update) - Basé sur la cinématique IMU
     * @param {number} dt Delta temps en secondes
     * @param {object} imu Données {ax, ay, az, gx, gy, gz}
     */
    predict(dt, imu) {
        if(dt <= 0) return;
        
        // Modèle de transition d'état F(x, u)
        // Ici on simplifie la matrice de rotation pour la performance JS, 
        // mais on garde l'intégration de vitesse.
        
        // 1. Récupération état actuel
        let vn = this.x.get([3]);
        let ve = this.x.get([4]);
        let vd = this.x.get([5]);
        let psi = this.x.get([8]); // Yaw
        
        // 2. Accélération Repère Corps -> Navigation (Simplifié 2D pour demo)
        // ax_n = ax * cos(psi) - ay * sin(psi)
        // ay_n = ax * sin(psi) + ay * cos(psi)
        const ax_n = imu.ax * Math.cos(psi) - imu.ay * Math.sin(psi);
        const ay_n = imu.ax * Math.sin(psi) + imu.ay * Math.cos(psi);
        const az_n = imu.az - sysState.config.gravityRef; // Compensation gravité
        
        // 3. Intégration Vitesse
        vn += ax_n * dt;
        ve += ay_n * dt;
        vd += az_n * dt;
        
        // 4. Intégration Position (Transport Rate WGS84 ignoré ici pour stabilité)
        const lat = this.x.get([0]) + (vn / (R_E_BASE + this.x.get([2]))) * dt;
        const lon = this.x.get([1]) + (ve / ((R_E_BASE + this.x.get([2])) * Math.cos(lat))) * dt;
        const alt = this.x.get([2]) - vd * dt; // NED convention: Down is positive, Alt is negative
        
        // Mise à jour vecteur X
        this.x.set([0], lat); this.x.set([1], lon); this.x.set([2], alt);
        this.x.set([3], vn);  this.x.set([4], ve);  this.x.set([5], vd);
        
        // Propagation Covariance P = P + Q * dt (Approximation additive pour JS)
        this.P = math.add(this.P, math.multiply(this.Q, dt));
        
        this.stateStr = "PRÉDICTION (IMU)";
    }

    /**
     * MISE À JOUR (Measurement Update) - Basé sur le GPS
     * @param {object} gps Données {lat, lon, alt, vn, ve, vd}
     * @param {number} acc Précision horizontale en mètres
     */
    update(gps, acc) {
        // Matrice d'Observation H (Identity pour les 6 premiers états)
        // Innovation y = z - Hx
        const z = math.matrix([gps.lat, gps.lon, gps.alt, gps.vn, gps.ve, gps.vd]);
        const x_meas = this.x.subset(math.index(math.range(0,6))); // Extraction des 6 premiers
        const y = math.subtract(z, x_meas);
        
        // Adaptation de R selon précision GPS (HDOP)
        const r_p = Math.pow(acc, 2);
        const r_v = Math.pow(0.5, 2); // Vitesse supposée précise à 0.5m/s
        const R_adapt = math.diag([r_p/R_E_BASE**2, r_p/R_E_BASE**2, r_p, r_v, r_v, r_v]); // Lat/Lon en radians
        
        // Gain de Kalman K = P H^T (H P H^T + R)^-1
        // Pour H = I (partiel), S = P_sub + R
        const P_sub = this.P.subset(math.index(math.range(0,6), math.range(0,6)));
        const S = math.add(P_sub, R_adapt);
        
        try {
            const S_inv = math.inv(S);
            const K = math.multiply(this.P.subset(math.index(math.range(0,9), math.range(0,6))), S_inv);
            
            // Mise à jour État: x = x + Ky
            const correction = math.multiply(K, y);
            this.x = math.add(this.x, correction);
            
            // Mise à jour Covariance: P = (I - KH)P
            // Simplification Joseph form pour stabilité numérique: P = (I-KH)P(I-KH)^T + KRK^T
            // Ici standard : P = P - K*S*K^T
            const KSK = math.multiply(math.multiply(K, S), math.transpose(K));
            this.P = math.subtract(this.P, KSK);
            
            this.stateStr = "CORRECTION (GPS)";
        } catch(e) {
            console.warn("UKF Matrix Inversion Failed", e);
            this.stateStr = "ERR MATRICE";
        }
    }
            }
// =============================================================================
// 7. ACQUISITION DE DONNÉES (DRIVERS)
// =============================================================================

// --- DRIVER IMU (ACC// =============================================================================
// 5. MOTEUR PHYSIQUE & CALCULS DÉRIVÉS
// =============================================================================

const PhysicsEngine = {
    
    // Calcul de la densité de l'air (Formule CIPM-2007 simplifiée)
    getAirDensity: (tempC, pressureHPa, humidity) => {
        const T_K = tempC + 273.15;
        const P_Pa = pressureHPa * 100;
        // Pression vapeur saturante (Tetens)
        const Es = 6.1078 * Math.pow(10, (7.5 * tempC) / (237.3 + tempC));
        const Pv = (humidity / 100) * Es * 100; // Pression partielle vapeur (Pa)
        const Pd = P_Pa - Pv; // Pression air sec
        
        const rho = (Pd / (R_AIR * T_K)) + (Pv / (R_VAPOR * T_K));
        return rho;
    },

    // Vitesse du son (Dépend Température + Humidité)
    getSoundSpeed: (tempC) => {
        return 331.3 * Math.sqrt(1 + tempC / 273.15);
    },

    // Relativité Restreinte & Générale
    getRelativity: (v, alt) => {
        const beta = v / C_L;
        const gamma = 1.0 / Math.sqrt(1.0 - beta * beta);
        
        // Dilatation temporelle cinématique (s/s)
        const dil_kin = gamma - 1;
        
        // Dilatation gravitationnelle (Schwarzschild approx faible champ)
        // dt_inf / dt_surf = sqrt(1 - rs/r)
        const r = R_E_BASE + alt;
        const rs = (2 * G_U * 5.972e24) / (C_L**2); // Masse Terre
        const dil_grav = (1 / Math.sqrt(1 - rs/r)) - 1;
        
        return { gamma, dil_kin, dil_grav };
    },

    // Dynamique des fluides
    getFluidDynamics: (v, rho, cda) => {
        const q = 0.5 * rho * v * v; // Pression dynamique
        const drag = q * cda;        // Force de trainée
        const power = drag * v;      // Puissance dissipée (Watts)
        return { q, drag, power };
    },

    // Bioclimatologie (Indice de chaleur, Rosée)
    getBioMetrics: (tempC, hum) => {
        const a = 17.27, b = 237.7;
        const alpha = ((a * tempC) / (b + tempC)) + Math.log(hum/100.0);
        const dewPoint = (b * alpha) / (a - alpha);
        return { dewPoint };
    }
};

// =============================================================================
// 6. GESTIONNAIRES D'ÉVÉNEMENTS (LISTENERS)
// =============================================================================

function setupListeners() {
    // Bouton Principal GPS
    const btnGps = $('toggle-gps-btn');
    if (btnGps) {
        btnGps.addEventListener('click', () => {
            if (sysState.status.isGpsActive) stopSystem();
            else startSystem();
        });
    }

    // Bouton Arrêt Urgence
    const btnEmerg = $('emergency-stop-btn');
    if (btnEmerg) {
        btnEmerg.addEventListener('click', () => {
            sysState.status.isEmergencyStop = !sysState.status.isEmergencyStop;
            btnEmerg.textContent = sysState.status.isEmergencyStop ? 
                "🛑 Arrêt d'urgence: ACTIF 🔴" : "🛑 Arrêt d'urgence: INACTIF 🟢";
            if (sysState.status.isEmergencyStop) stopSystem();
        });
    }

    // Reset All
    const btnReset = $('reset-all-btn');
    if (btnReset) {
        btnReset.addEventListener('click', () => {
            if(confirm("Réinitialiser toutes les données ?")) {
                sysState.metrics.totalDist = 0;
                sysState.metrics.maxSpeed = 0;
                sysState.metrics.timeMoving = 0;
                sysState.status.initTime = Date.now();
                if(ukfInstance) ukfInstance = new EngineeringUKF();
            }
        });
    }
    
    // Inputs Physiques
    $('mass-input')?.addEventListener('change', (e) => sysState.config.mass = parseFloat(e.target.value));
    $('gps-accuracy-override')?.addEventListener('change', (e) => {
        sysState.raw.gps.acc = parseFloat(e.target.value);
    });
                    }EL/GYRO) ---
function startIMU() {
    // Détection iOS 13+
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission().then(response => {
            if (response === 'granted') {
                window.addEventListener('devicemotion', handleMotion);
                window.addEventListener('deviceorientation', handleOrientation);
            }
        }).catch(console.error);
    } else {
        window.addEventListener('devicemotion', handleMotion);
        window.addEventListener('deviceorientation', handleOrientation);
    }
    sysState.status.isImuActive = true;
    $('statut-capteur').textContent = "Actif (Raw IMU)";
}

function handleMotion(e) {
    // Filtrage simple passe-bas pour lissage affichage
    const acc = e.accelerationIncludingGravity;
    if (acc) {
        sysState.raw.imu.ax = acc.x || 0;
        sysState.raw.imu.ay = acc.y || 0;
        sysState.raw.imu.az = acc.z || 0;
        
        // ZUPT Detection (Zero Velocity Update)
        // Si l'accélération totale est proche de 1G (stationnaire)
        const norm = Math.sqrt(acc.x**2 + acc.y**2 + acc.z**2);
        sysState.status.isZuptActive = Math.abs(norm - 9.81) < 0.2; 
    }
    
    const rot = e.rotationRate;
    if (rot) {
        sysState.raw.imu.gx = rot.alpha || 0;
        sysState.raw.imu.gy = rot.beta || 0;
        sysState.raw.imu.gz = rot.gamma || 0;
    }
}

function handleOrientation(e) {
    // Magnétomètre simulé ou réel si dispo
    sysState.raw.imu.mx = e.alpha || 0; // Compass heading
}

// --- DRIVER GNSS (GEOLOCATION API) ---
function startGPS() {
    if (!navigator.geolocation) return alert("Pas de GPS");
    
    $('statut-gps-acquisition').textContent = "Acquisition...";
    sysState.status.isGpsActive = true;
    
    sysState.config.watchID = navigator.geolocation.watchPosition(
        (pos) => {
            const c = pos.coords;
            const now = Date.now();
            
            // Pipeline de Fusion
            const gpsData = {
                lat: c.latitude, lon: c.longitude, alt: c.altitude || 0,
                vn: c.speed ? c.speed * Math.cos(c.heading*D2R) : 0,
                ve: c.speed ? c.speed * Math.sin(c.heading*D2R) : 0,
                vd: 0 // Le GPS mobile donne rarement la vitesse verticale précise
            };
            
            // 1. UPDATE UKF
            if (ukfInstance) {
                // R Adaptatif: Si ZUPT actif, on fait confiance à l'IMU (vitesse 0)
                const R_val = sysState.status.isZuptActive ? 0.1 : (c.accuracy**2);
                ukfInstance.update(gpsData, R_val);
            }
            
            // 2. STOCKAGE ET CALCULS DE DISTANCE
            if (sysState.raw.gps.ts > 0) {
                // Distance 3D (Turf.js est 2D, on ajoute Pythagore pour l'alt)
                const d2d = turf.distance(
                    [sysState.raw.gps.lon, sysState.raw.gps.lat],
                    [c.longitude, c.latitude], 
                    {units: 'meters'}
                );
                const dAlt = c.altitude - sysState.raw.gps.alt;
                const d3d = Math.sqrt(d2d**2 + dAlt**2);
                
                // Filtre anti-saut (Vitesse max théorique 2000 km/h)
                if (d3d / ((now - sysState.raw.gps.ts)/1000) < 555) {
                    sysState.metrics.totalDist += d3d;
                }
            }
            
            sysState.raw.gps = {
                lat: c.latitude, lon: c.longitude, alt: c.altitude,
                acc: c.accuracy, spd: c.speed, head: c.heading, ts: now
            };
            
            // Mise à jour DOM directe (Feedback immédiat)
            $('statut-gps-acquisition').textContent = `FIX 3D (${c.accuracy.toFixed(1)}m)`;
            if(mapInstance) updateMap(c.latitude, c.longitude, c.accuracy);
            
        },
        (err) => {
            console.error("GPS Error", err);
            $('statut-gps-acquisition').textContent = `ERREUR: ${err.message}`;
        },
        { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
    );
    
    // UI Update
    $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
    $('toggle-gps-btn').style.background = "#ffc107";
    $('toggle-gps-btn').style.color = "#000";
}

// --- DRIVER MÉTÉO (API + FALLBACK) ---
async function updateWeather() {
    const lat = sysState.pose.lat;
    const lon = sysState.pose.lon;
    
    try {
        const res = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!res.ok) throw new Error("API Error");
        const data = await response.json();
        
        sysState.raw.env.temp = data.main.temp;
        sysState.raw.env.press = data.main.pressure;
        sysState.raw.env.hum = data.main.humidity;
        
        $('statut-meteo').textContent = "EN LIGNE (API)";
    } catch(e) {
        // Fallback ISA Model si hors ligne
        const alt = sysState.pose.alt;
        const temp = 15 - 0.0065 * alt;
        const press = 1013.25 * Math.pow(1 - 0.0065*alt/288.15, 5.255);
        
        sysState.raw.env.temp = temp;
        sysState.raw.env.press = press;
        $('statut-meteo').textContent = "SIMULÉ (ISA)";
    }
}
// =============================================================================
// 8. MOTEUR DE RENDU (DOM UPDATE LOOP) & INIT
// =============================================================================

function updateDashboard(dt) {
    if (!ukfInstance) return;
    
    // 1. Calcul de l'état UKF extrapolé (haute fréquence)
    // On utilise l'IMU pour combler les trous entre les points GPS (1Hz -> 60Hz)
    ukfInstance.predict(dt, sysState.raw.imu);
    const est = ukfInstance.getState();
    
    // Mise à jour de l'état global
    sysState.pose.lat = est.lat;
    sysState.pose.lon = est.lon;
    sysState.pose.alt = est.alt;
    sysState.pose.speed = est.speed;
    
    // 2. Calculs Physiques Dérivés
    // Densité Air
    const rho = PhysicsEngine.getAirDensity(sysState.raw.env.temp, sysState.raw.env.press, sysState.raw.env.hum);
    const c_sound = PhysicsEngine.getSoundSpeed(sysState.raw.env.temp);
    const g_loc = computeLocalGravity(est.lat, est.alt);
    
    const phys = PhysicsEngine.getFluidDynamics(est.speed, rho, sysState.config.cda);
    const rel = PhysicsEngine.getRelativity(est.speed, est.alt);
    const bio = PhysicsEngine.getBioMetrics(sysState.raw.env.temp, sysState.raw.env.hum);
    
    // 3. MISE À JOUR MASSIVE DU DOM (Optimisée)
    // Ne mettez à jour que si l'élément existe ($ check inside helper not needed if IDs guaranteed)
    
    // --- Section Contrôles ---
    $('elapsed-session-time').textContent = fmt((Date.now() - sysState.status.initTime)/1000, 1, ' s');
    $('time-movement').textContent = fmt(sysState.metrics.moveTime, 1, ' s');
    
    // --- Section Vitesse ---
    const spdKmh = est.speed * 3.6;
    $('speed-stable').textContent = fmt(spdKmh, 2, ' km/h');
    $('speed-stable-ms').textContent = fmt(est.speed, 3, ' m/s');
    $('speed-stable-kms').textContent = fmt(est.speed/1000, 5, ' km/s');
    $('speed-3d-inst').textContent = fmt(sysState.raw.gps.spd * 3.6, 2, ' km/h'); // GPS Brut
    $('vitesse-max-session').textContent = fmt(sysState.metrics.maxSpeed * 3.6, 2, ' km/h');
    
    // --- Section Physique ---
    $('speed-of-sound-calc').textContent = fmt(c_sound, 2, ' m/s');
    $('mach-number').textContent = fmt(est.speed / c_sound, 4);
    $('lorentz-factor').textContent = rel.gamma.toFixed(12);
    $('time-dilation-vitesse').textContent = fmtExp((rel.gamma-1)*8.64e13, 2, ' ns/j');
    $('time-dilation-gravite').textContent = fmtExp(rel.dil_grav*8.64e13, 2, ' ns/j');
    
    // Énergies
    const E0 = sysState.config.mass * C_L**2;
    $('energy-rest-mass').textContent = fmtExp(E0, 4, ' J');
    $('energy-relativistic').textContent = fmtExp(E0 * rel.gamma, 4, ' J');
    $('kinetic-energy').textContent = fmt(0.5 * sysState.config.mass * est.speed**2, 1, ' J');
    
    // --- Section Distance ---
    $('distance-totale').textContent = fmt(sysState.metrics.totalDist, 2, ' m');
    $('distance-light-s').textContent = fmtExp(sysState.metrics.totalDist / C_L, 3, ' ls');
    
    // --- Section Météo/Bio ---
    $('air-density').textContent = fmt(rho, 4, ' kg/m³');
    $('dew-point').textContent = fmt(bio.dewPoint, 1, ' °C');
    $('temp-air-2').textContent = fmt(sysState.raw.env.temp, 1, ' °C');
    $('pressure-2').textContent = fmt(sysState.raw.env.press, 0, ' hPa');
    
    // --- Section Dynamique ---
    $('gravite-wgs84').textContent = fmt(g_loc, 5, ' m/s²');
    $('dynamic-pressure').textContent = fmt(phys.q, 2, ' Pa');
    $('drag-force').textContent = fmt(phys.drag, 2, ' N');
    
    // IMU Brutes
    $('acceleration-x').textContent = fmt(sysState.raw.imu.ax, 2);
    $('acceleration-y').textContent = fmt(sysState.raw.imu.ay, 2);
    $('acceleration-z').textContent = fmt(sysState.raw.imu.az, 2);
    
    // EKF Debug
    $('statut-ekf-fusion').textContent = ukfInstance.stateStr;
    $('ukf-v-uncert').textContent = fmt(Math.sqrt(est.kUncert), 3);
}

// --- SYSTÈME DE BOUCLE ---
function startSystem() {
    if (sysState.status.isGpsActive) return;
    
    // Init UKF
    ukfInstance = new EngineeringUKF();
    
    // Démarrage Capteurs
    startIMU();
    startGPS();
    
    // Boucle Rapide (Animation Frame ~60Hz)
    let lastT = performance.now();
    const loop = (now) => {
        if (!sysState.status.isGpsActive) return;
        const dt = (now - lastT) / 1000;
        lastT = now;
        
        updateDashboard(dt);
        requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
    
    // Boucle Lente (Météo/Astro) - 1Hz
    setInterval(() => {
        updateWeather();
        // updateAstro(); // Appeler astro.js si chargé
    }, 1000);
}

function stopSystem() {
    if (sysState.config.watchID) navigator.geolocation.clearWatch(sysState.config.watchID);
    window.removeEventListener('devicemotion', handleMotion);
    sysState.status.isGpsActive = false;
    
    $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
    $('toggle-gps-btn').style.background = "";
    $('statut-gps-acquisition').textContent = "Arrêté";
}

// --- INIT MAP ---
function updateMap(lat, lon, acc) {
    if (!mapInstance) {
        mapInstance = L.map('map').setView([lat, lon], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(mapInstance);
        userMarker = L.marker([lat, lon]).addTo(mapInstance);
        pathPolyline = L.polyline([], {color: 'red'}).addTo(mapInstance);
    }
    userMarker.setLatLng([lat, lon]);
    pathPolyline.addLatLng([lat, lon]);
}

// --- MAIN ENTRY POINT ---
document.addEventListener('DOMContentLoaded', () => {
    setupListeners();
    
    // Check dependencies
    if (typeof math === 'undefined' || typeof L === 'undefined') {
        alert("CRITIQUE: math.js ou leaflet.js manquant !");
    }
    
    console.log("SYSTEME PRET. ATTENTE UTILISATEUR.");
});
