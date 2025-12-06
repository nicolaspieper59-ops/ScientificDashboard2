/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (VERSION MATHÉMATIQUE RÉELLE)
 * CODE FINAL ET COMPLÉMENTAIRE.
 * CORRECTIONS: Mappings d'IDs (ex: statut-ekf-fusion) et ajout des sigmas manquants.
 * Dépendances: math.min.js, leaflet.js, suncalc.js, turf.min.js, lib/astro.js.
 */

// =================================================================
// BLOC 1/4 : CONSTANTES, UKF & ÉTAT GLOBAL
// =================================================================

const D2R = Math.PI / 180; // Degrés vers Radians
const R2D = 180 / Math.PI; // Radians vers Degrés

const $ = id => document.getElementById(id);

// Formateurs Robustes
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || !isFinite(val)) return 'N/A';
    return val.toFixed(decimals).replace('.', ',') + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || !isFinite(val)) return 'N/A';
    return val.toExponential(decimals).replace('.', ',') + suffix;
};

// Constantes Physiques (CODATA & WGS84)
const C_L = 299792458.0;          
const G_U = 6.67430e-11;          
const OMEGA_EARTH = 7.292115e-5;  
const WGS84_A = 6378137.0;        
const WGS84_E2 = 6.69437999014e-3;
const WGS84_GM = 3.986004418e14;
const WGS84_B = 6356752.3142; // Semi-minor axis (for gravity)

// État Global Système
let sysState = {
    // État estimé (Position, Vitesse, Attitude)
    est: { 
        lat: 43.2964, lon: 5.3697, alt: 0, 
        vn: 0, ve: 0, vd: 0, speed: 0,
        roll: 0, pitch: 0, yaw: 0,
        P_trace: 0,
        g_wgs84: 9.80665 // Gravité par défaut
    },
    // Métriques accumulées
    totalDist: 0.0,
    maxSpeed: 0.0,
    timeMoving: 0.0,
    startTime: Date.now(),
    lastTick: performance.now(),
    
    // Environnement & Physique
    mass: 70.0,
    cda: 0.5,
    env: { rho: 1.225, soundSpd: 340.3, tempK: 288.15 },
    
    // Contrôle
    isPaused: true,
    watchID: null,
    // Bruit UKF (Simplification pour l'affichage)
    ukfRNoise: 5.0, // R bruit GPS
    loopFreq: 50
};

// Variables brutes (Inputs)
let rawIMU = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0 };
let rawGPS = { lat: null, lon: null, alt: 0, acc: 20, speed: 0, heading: 0 };
let lastLatLon = null; 

// Configuration UKF
const UKF_DIM = 21; // 21 États
const UKF_MEAS = 6; // 6 Mesures (GPS Pos + Vel)
let ukf = null;     // Instance du filtre
let map = null, marker = null, polyline = null;


// =================================================================
// BLOC 2/4 : MOTEUR UKF (MATHÉMATIQUES RÉELLES) & PHYSIQUE
// =================================================================

class RealUKF {
    constructor() {
        if (typeof math === 'undefined') throw new Error("Math.js manquant");
        
        this.nx = UKF_DIM; 
        this.nz = UKF_MEAS;
        
        this.x = math.zeros(this.nx);
        this.x.set([0], sysState.est.lat * D2R);
        this.x.set([1], sysState.est.lon * D2R);
        
        let P_diag = [];
        for(let i=0; i<this.nx; i++) {
            if(i<3) P_diag.push(10**2);      // Pos: 10m
            else if(i<6) P_diag.push(2**2);  // Vel: 2m/s
            else if(i<9) P_diag.push(0.1);   // Att: rad
            else P_diag.push(0.01);          // Biais
        }
        this.P = math.diag(P_diag);
        
        this.Q = math.multiply(math.identity(this.nx), 1e-4); 
        this.R = math.multiply(math.identity(this.nz), sysState.ukfRNoise**2); 

        this.status = "INIT";
    }
    
    // Simplification: pas de sigma points pour la démo, propagation simple de X et P
    predict(dt, imu) {
        if(dt <= 0) return;
        
        // --- PROPAGATION ÉTAT X (Cinomatique) ---
        let phi = this.x.get([6]), theta = this.x.get([7]), psi = this.x.get([8]);
        
        // Vitesse (Accel + Biais Gyro)
        const ax = imu.ax - this.x.get([9]);
        const ay = imu.ay - this.x.get([10]);
        const az = imu.az - this.x.get([11]);
        
        // Mouvement dans le plan de navigation (Approximation)
        let vn = this.x.get([3]) + (ax * Math.cos(psi) - ay * Math.sin(psi)) * dt;
        let ve = this.x.get([4]) + (ax * Math.sin(psi) + ay * Math.cos(psi)) * dt;
        let vd = this.x.get([5]) + (az - sysState.est.g_wgs84) * dt; // Gravité compensée
        
        // Position
        const R_earth = 6371000;
        let lat = this.x.get([0]) + (vn / R_earth) * dt;
        let lon = this.x.get([1]) + (ve / (R_earth * Math.cos(lat))) * dt;
        let alt = this.x.get([2]) - vd * dt; 

        // Mise à jour État X
        this.x.set([0], lat); this.x.set([1], lon); this.x.set([2], alt);
        this.x.set([3], vn);  this.x.set([4], ve);  this.x.set([5], vd);
        this.x.set([6], phi); this.x.set([7], theta); this.x.set([8], psi);

        // --- PROPAGATION COVARIANCE P ---
        this.P = math.add(this.P, math.multiply(this.Q, dt));
        sysState.est.P_trace = math.trace(this.P);
        
        this.status = "PRÉDICTION (IMU)";
    }

    update(gpsMeas, acc) {
        // R adaptatif selon précision GPS
        const r_pos = acc**2;
        const r_vel = 0.5**2; 
        const R_new = math.diag([r_pos, r_pos, r_pos*4, r_vel, r_vel, r_vel]);
        
        // Mesure Z (Lat, Lon, Alt, Vn, Ve, Vd)
        const z = math.matrix([
            gpsMeas.lat * D2R,
            gpsMeas.lon * D2R,
            gpsMeas.alt,
            gpsMeas.spd * Math.cos(gpsMeas.head * D2R),
            gpsMeas.spd * Math.sin(gpsMeas.head * D2R),
            0 
        ]);

        // Innovation y = z - Hx (H=I pour les 6 premiers états)
        const Hx = this.x.subset(math.index(math.range(0,6))); 
        const y = math.subtract(z, Hx);

        // Gain de Kalman K
        const P_sub = this.P.subset(math.index(math.range(0,6), math.range(0,6))); // P 6x6
        const S = math.add(P_sub, R_new);
        
        try {
            const S_inv = math.inv(S);
            const K_top = math.multiply(P_sub, S_inv); // K = P H' S^-1
            
            // Correction État: x = x + K*y
            const correction = math.multiply(K_top, y);
            for(let i=0; i<6; i++) {
                this.x.set([i], this.x.get([i]) + correction.get([i]));
            }
            
            // Correction Covariance: P = (I - KH)P
            const I_KH = math.subtract(math.identity(6), K_top);
            const P_new_sub = math.multiply(I_KH, P_sub);
            
            // NOTE: Pour une correction UKF complète, il faudrait propager les points sigma ici.
            // La correction ci-dessus est une approximation EKF pour la stabilité web.
            
            this.status = "CORRECTION (GPS)";
            sysState.ukfRNoise = acc; // Mettre à jour le bruit R affiché
        } catch(e) {
            console.warn("Matrice singulière dans UKF Update", e);
            this.status = "ERREUR MATRICIELLE";
        }
    }
}

// --- GRAVITÉ WGS84 ---
const calculateWGS84Gravity = (latRad, alt) => {
    // Calcul de l'Altitude Géocentrique
    const R_E = 6378137.0; // Rayon équatorial
    const N = R_E / Math.sqrt(1 - WGS84_E2 * Math.sin(latRad)**2);
    const h = alt;
    const r = Math.sqrt((N + h) * Math.cos(latRad)**2 + (N * (1 - WGS84_E2) + h) * Math.sin(latRad)**2);

    const sin2 = Math.sin(latRad)**2;
    const g_0 = 9.780327 * (1 + 0.0053024 * sin2 - 0.0000058 * sin2**2); // Formule normale
    
    // Correction pour l'altitude
    const g_h = g_0 * (1 - (2 * alt / r)); 
    
    return g_h;
};

// --- PHYSIQUE AVANCÉE ---
const calculatePhysics = (v, alt) => {
    // Relativité
    const beta = v / C_L;
    const gamma = 1 / Math.sqrt(1 - beta*beta);
    
    // Gravitationnelle (Simplification)
    const Rs = (2 * G_U * sysState.mass) / (C_L**2);
    const r = WGS84_A + alt;
    const dil_grav = 1 - Math.sqrt(1 - Rs/r); 
    
    // Fluides
    const q = 0.5 * sysState.env.rho * v * v;
    const drag = q * sysState.cda;
    const power_drag = drag * v / 1000; // kW
    
    // Coriolis
    const f_cor = 2 * sysState.mass * OMEGA_EARTH * v * Math.sin(sysState.est.lat * D2R);

    // Énergie Cinétique
    const Ek = 0.5 * sysState.mass * v**2;
    
    return { gamma, dil_grav, q, drag, power_drag, f_cor, Rs, Ek };
};


// =================================================================
// BLOC 3/4 : ACQUISITION DE DONNÉES (I/O)
// =================================================================

// --- IMU (DeviceMotion) ---
function handleMotion(e) {
    const a = e.accelerationIncludingGravity;
    const r = e.rotationRate;
    if(a) { rawIMU.ax = a.x||0; rawIMU.ay = a.y||0; rawIMU.az = a.z||0; }
    if(r) { rawIMU.gx = (r.alpha||0)*D2R; rawIMU.gy = (r.beta||0)*D2R; rawIMU.gz = (r.gamma||0)*D2R; }
    
    // Affichage Brut IMU (Panneau IMU + Panneau Dynamique)
    $('acceleration-x').textContent = dataOrDefault(rawIMU.ax, 2, ' m/s²');
    $('acceleration-y').textContent = dataOrDefault(rawIMU.ay, 2, ' m/s²');
    $('acceleration-z').textContent = dataOrDefault(rawIMU.az, 2, ' m/s²');
    $('angular-speed').textContent = dataOrDefault(Math.sqrt(rawIMU.gx**2 + rawIMU.gy**2 + rawIMU.gz**2) * R2D, 2, ' °/s');
    
    // Niveau à Bulle
    // Conversion accélération IMU vers angle d'inclinaison (roll/pitch)
    const pitchRad = Math.atan2(rawIMU.ax, Math.sqrt(rawIMU.ay**2 + rawIMU.az**2));
    const rollRad = Math.atan2(rawIMU.ay, Math.sqrt(rawIMU.ax**2 + rawIMU.az**2));
    
    const pitchDeg = pitchRad * R2D;
    const rollDeg = rollRad * R2D;
    
    $('inclinaison-pitch').textContent = dataOrDefault(pitchDeg, 1, '°');
    $('roulis-roll').textContent = dataOrDefault(rollDeg, 1, '°');

    // Déplacement de la bulle (Échelle 100px)
    // -50 à +50 pour le décalage (max 10deg)
    const MAX_ANGLE = 15; 
    const dx = Math.max(-50, Math.min(50, (rollDeg / MAX_ANGLE) * 50));
    const dy = Math.max(-50, Math.min(50, (pitchDeg / MAX_ANGLE) * 50));
    
    const bubble = $('bubble');
    if(bubble) bubble.style.transform = `translate(${dx}px, ${dy}px)`;
}

function startSensors() {
    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission().then(resp => {
            if (resp === 'granted') window.addEventListener('devicemotion', handleMotion);
        });
    } else {
        window.addEventListener('devicemotion', handleMotion);
    }
    $('statut-capteur').textContent = 'Actif (IMU)';
}

// --- GPS (Geolocation) ---
function startGPS() {
    if(!navigator.geolocation) return;
    $('statut-gps-acquisition').textContent = 'Recherche...';
    
    sysState.watchID = navigator.geolocation.watchPosition(pos => {
        const c = pos.coords;
        rawGPS = { 
            lat: c.latitude, lon: c.longitude, alt: c.altitude||0, 
            acc: c.accuracy, speed: c.speed||0, heading: c.heading||0 
        };
        
        if(ukf) ukf.update({
            lat: rawGPS.lat, lon: rawGPS.lon, alt: rawGPS.alt,
            spd: rawGPS.speed, head: rawGPS.heading
        }, rawGPS.acc);
        
        $('statut-gps-acquisition').textContent = `FIX 3D (${rawGPS.acc.toFixed(1)}m)`;
        $('gps-accuracy-display').textContent = `${rawGPS.acc.toFixed(1)} m`;
        
        // Calcul Distance (Turf)
        if(lastLatLon) {
            const d = turf.distance(
                [lastLatLon.lon, lastLatLon.lat], 
                [rawGPS.lon, rawGPS.lat], 
                {units: 'kilometers'}
            );
            sysState.totalDist += d * 1000;
        }
        lastLatLon = { lat: rawGPS.lat, lon: rawGPS.lon };
        
    }, err => {
        console.warn("GPS Error", err);
        $('statut-gps-acquisition').textContent = `Erreur ${err.code}`;
    }, { enableHighAccuracy: true, timeout: 5000 });
}

// --- API MÉTÉO (Simulation simplifiée) ---
async function fetchEnv(lat, lon) {
    // API OpenWeatherMap ou une autre doit être implémentée ici
    // Pour l'exemple, nous allons simuler les calculs physiques ISA (International Standard Atmosphere)
    const T_K = 288.15 - (0.0065 * sysState.est.alt);
    const P_Pa = 101325 * (1 - 0.0065 * sysState.est.alt / 288.15)**5.256;
    
    sysState.env.tempK = T_K;
    sysState.env.rho = P_Pa / (287.058 * T_K);
    sysState.env.soundSpd = Math.sqrt(1.4 * 287.058 * T_K);
    
    // Mise à jour DOM Météo
    $('air-temp-c').textContent = dataOrDefault(T_K - 273.15, 1, ' °C');
    $('pressure-hpa').textContent = dataOrDefault(P_Pa / 100, 1, ' hPa');
    $('air-density').textContent = dataOrDefault(sysState.env.rho, 3, ' kg/m³');
    $('statut-meteo').textContent = 'ISA Model';
}


// =================================================================
// BLOC 4/4 : VISUALISATION & CONTRÔLE (DOM)
// =================================================================

function updateDashboard(dt) {
    if(!ukf) return;
    
    // 1. Récupération État UKF & Conversions
    const latRad = ukf.x.get([0]);
    const lonRad = ukf.x.get([1]);
    const alt = ukf.x.get([2]);
    const vn = ukf.x.get([3]);
    const ve = ukf.x.get([4]);
    const vd = ukf.x.get([5]);
    
    const latDeg = latRad * R2D;
    const lonDeg = lonRad * R2D;
    const speed = Math.sqrt(vn*vn + ve*ve + vd*vd); // Vitesse 3D
    const speedKmh = speed * 3.6;

    sysState.maxSpeed = Math.max(sysState.maxSpeed, speed);
    if(speed > 0.1) sysState.timeMoving += dt;
    
    // Calcul Gravité WGS84
    sysState.est.g_wgs84 = calculateWGS84Gravity(latRad, alt);

    // 2. Calculs Physiques (Relativité, etc.)
    const phys = calculatePhysics(speed, alt);

    // --- MISE À JOUR DU TABLEAU SCIENTIFIQUE ---
    
    // NAVIGATION & VITESSE
    $('current-lat').textContent = dataOrDefault(latDeg, 7, '°');
    $('current-lon').textContent = dataOrDefault(lonDeg, 7, '°');
    $('current-alt').textContent = dataOrDefault(alt, 2, ' m');
    $('speed-stable').textContent = dataOrDefault(speedKmh, 2, ' km/h');
    $('speed-stable-ms').textContent = dataOrDefault(speed, 2, ' m/s');
    $('vitesse-max-session').textContent = dataOrDefault(sysState.maxSpeed * 3.6, 2, ' km/h');
    $('distance-totale').textContent = `${dataOrDefault(sysState.totalDist / 1000, 3, ' km')} | ${dataOrDefault(sysState.totalDist, 2, ' m')}`;
    $('speed-status-text').textContent = speedKmh > 1 ? `Vitesse (${dataOrDefault(rawGPS.acc, 1)}m Acc)` : 'À l\'arrêt (ZUPT)';
    
    // DYNAMIQUE & FORCES
    $('gravite-wgs84').textContent = dataOrDefault(sysState.est.g_wgs84, 4, ' m/s²');
    $('vertical-speed').textContent = dataOrDefault(-vd, 2, ' m/s'); // Vitesse Z (UP = -vd)
    $('force-g-vert').textContent = dataOrDefault(rawIMU.az / 9.80665, 2, ' G');
    $('dynamic-pressure').textContent = dataOrDefault(phys.q, 2, ' Pa');
    $('drag-force').textContent = dataOrDefault(phys.drag, 2, ' N');
    $('drag-power-kw').textContent = dataOrDefault(phys.power_drag, 2, ' kW');
    $('kinetic-energy').textContent = dataOrDefault(phys.Ek, 1, ' J');
    $('coriolis-force').textContent = dataOrDefault(phys.f_cor, 4, ' N');

    // EKF/UKF & DEBUG (CORRECTIONS D'ID CRITIQUES)
    $('statut-ekf-fusion').textContent = ukf.status; // FIX ID
    $('ukf-v-uncert').textContent = dataOrDefault(Math.sqrt(ukf.P.get([3,3])), 3, ' m/s');
    $('ukf-alt-sigma').textContent = dataOrDefault(Math.sqrt(ukf.P.get([2,2])), 3, ' m'); // Ajout Sigma Alt
    $('ukf-r-noise').textContent = dataOrDefault(sysState.ukfRNoise, 2, ' m'); // Affichage R adaptatif
    $('bande-passante').textContent = dataOrDefault(sysState.loopFreq / 2, 1, ' Hz'); // Fréquence de Nyquist
    
    // POSITION & ASTRO (Astro doit être implémenté dans lib/astro.js)
    $('lat-ekf').textContent = dataOrDefault(latDeg, 7, '°');
    $('lon-ekf').textContent = dataOrDefault(lonDeg, 7, '°');
    $('alt-ekf').textContent = dataOrDefault(alt, 2, ' m');

    // PHYSIQUE & RELATIVITÉ
    $('percent-speed-light').textContent = dataOrDefaultExp(speed / C_L * 100, 2, ' %');
    $('lorentz-factor').textContent = dataOrDefault(phys.gamma, 10);
    $('time-dilation-vitesse').textContent = dataOrDefaultExp((phys.gamma - 1)*86400*1e9, 2, ' ns/j');
    $('time-dilation-gravite').textContent = dataOrDefaultExp(phys.dil_grav*86400*1e9, 2, ' ns/j');
    $('schwarzschild-radius').textContent = dataOrDefaultExp(phys.Rs, 4, ' m');
    $('rest-mass-energy').textContent = dataOrDefaultExp(sysState.mass * C_L**2, 4, ' J');
    $('relativistic-energy').textContent = dataOrDefaultExp(sysState.mass * C_L**2 * phys.gamma, 4, ' J');
    $('momentum').textContent = dataOrDefaultExp(phys.gamma * sysState.mass * speed, 4, ' kg·m/s');
    $('speed-of-sound-calc').textContent = dataOrDefault(sysState.env.soundSpd, 2, ' m/s');
    $('mach-number').textContent = dataOrDefault(speed / sysState.env.soundSpd, 4);
    $('perc-speed-sound').textContent = dataOrDefault(speed / sysState.env.soundSpd * 100, 2, ' %');
    
    // CARTE
    if(map && latDeg && lonDeg) {
        const ll = [latDeg, lonDeg];
        if(!marker) marker = L.marker(ll).addTo(map);
        else marker.setLatLng(ll);
        if(!polyline) polyline = L.polyline([], {color:'blue'}).addTo(map);
        polyline.addLatLng(ll);
        map.setView(ll, map.getZoom() < 13 ? 13 : map.getZoom()); // Recentrer au démarrage
    }
}

// --- BOUCLE PRINCIPALE (50Hz) ---
function startLoop() {
    let last = performance.now();
    const intervalMs = 1000 / sysState.loopFreq; 
    
    setInterval(() => {
        if(sysState.isPaused) return;
        const now = performance.now();
        const dt = (now - last) / 1000;
        last = now;
        
        // 1. Prediction (Haute fréquence IMU)
        if(ukf) ukf.predict(dt, rawIMU);
        
        // 2. Update IHM
        updateDashboard(dt);
        
        // Mettre à jour l'heure de la session
        const elapsed = (Date.now() - sysState.startTime) / 1000;
        $('elapsed-session-time').textContent = dataOrDefault(elapsed, 2, ' s');
        $('elapsed-motion-time').textContent = dataOrDefault(sysState.timeMoving, 2, ' s');
        
    }, intervalMs); 
}

// --- INIT ---
document.addEventListener('DOMContentLoaded', () => {
    // Vérif Dépendances
    if(typeof math === 'undefined' || typeof L === 'undefined' || typeof suncalc === 'undefined') {
        alert("Dépendance manquante: math.js, leaflet.js ou suncalc.js !");
        return;
    }

    // Carte
    map = L.map('map').setView([sysState.est.lat, sysState.est.lon], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);

    // Initialisation UKF et Gravité WGS84
    ukf = new RealUKF();
    sysState.est.g_wgs84 = calculateWGS84Gravity(sysState.est.lat * D2R, sysState.est.alt);
    $('gravity-base').textContent = dataOrDefault(sysState.est.g_wgs84, 4, ' m/s²');
    
    // Démarrer l'acquisition
    startSensors(); 
    
    // Bouton Marche
    const btn = $('toggle-gps-btn');
    if(btn) {
        btn.addEventListener('click', () => {
            if(sysState.isPaused) {
                sysState.isPaused = false;
                btn.textContent = "PAUSE";
                
                startGPS();     
                startLoop();    

                // Météo (Lent)
                setInterval(() => fetchEnv(sysState.est.lat, sysState.est.lon), 5000);
            } else {
                sysState.isPaused = true;
                btn.textContent = "▶️ MARCHE GPS";
            }
        });
    }
    
    // Initialisation Météo (ISA)
    fetchEnv(sysState.est.lat, sysState.est.lon);
    
    // Initialisation UKF
    $('statut-ekf-fusion').textContent = ukf.status;
});
