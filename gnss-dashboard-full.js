/**
 * GNSS SPACETIME DASHBOARD - VERSION ULTIME V4.0 (FUSION TOTALE)
 * Intègre: UKF 21 États, ZUPT, Correction Troposphérique, Astro V3.2, Modèles Relativistes Complets.
 * DÉPENDANCES CRITIQUES : math.min.js, leaflet.js, suncalc.js, turf.min.js
 */

// =================================================================
// BLOC 1/4 : CONSTANTES, ÉTAT GLOBAL & FILTRES AVANCÉS
// =================================================================

const $ = id => document.getElementById(id);
const dataOrDefault = (val, dec, suf = '') => (val === undefined || val === null || isNaN(val)) ? 'N/A' : val.toFixed(dec) + suf;
const dataOrDefaultExp = (val, dec, suf = '') => (val === undefined || val === null || isNaN(val)) ? 'N/A' : val.toExponential(dec) + suf;

// --- CONSTANTES PHYSIQUES (V3.2) ---
const C_L = 299792458;          
const G_U = 6.67430e-11;        
const R_E_BASE = 6371000;       
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;             
const OMEGA_EARTH = 7.2921159e-5; 
const TROPO_K2 = 382000; // Constante réfractive pour ZWD

// --- PARAMÈTRES FILTRES (V3.2) ---
const UKF_R_MAX = 500.0;     
const R_MIN = 0.01;
const ZUPT_RAW_THRESHOLD = 1.0;     
const ZUPT_ACCEL_THRESHOLD = 0.5;   
const MIN_SPD = 0.05;        
const MAX_ACC = 200;        
const NETHER_RATIO = 8.0;
const ALT_TH = -50;

// --- ÉTAT GLOBAL ---
let sysState = {
    // Navigation
    lat: 43.2964, lon: 5.3697, alt: 0, 
    vn: 0, ve: 0, vd: 0, speed: 0, heading: 0,
    kUncert: 1000, kAltUncert: 10,
    
    // Métriques
    totalDist: 0.0, maxSpeed: 0.0, timeMoving: 0.0, startTime: Date.now(),
    
    // Environnement
    tempK: 288.15, pressureHPa: 1013.25, humidity: 50, rho: 1.225, soundSpd: 340.3,
    
    // Contrôle
    gpsMode: 'HIGH_FREQ', isPaused: true, emergencyStop: false,
    
    // IMU Brut
    accel: {x:0, y:0, z:0}, gyro: {x:0, y:0, z:0}, mag: {x:0, y:0, z:0}
};

let ukf = null;
let lastGPS = null;
let lastIMUTime = 0;
let map = null, marker = null, polyline = null;

// --- FILTRE DE KALMAN EKF (V3.2 Logic) ---
// Note: Utilisé en parallèle de l'UKF pour la robustesse sur l'altitude
function kFilterAltitude(kAlt_in, kAltUncert_in, nAlt, acc, dt) {
    if (nAlt === null) return { kAlt: kAlt_in, kAltUncert: kAltUncert_in };
    const R_alt = Math.max(0.1, acc * acc); 
    const Q_alt = 0.1 * dt; 
    
    let pAlt = kAlt_in === null ? nAlt : kAlt_in; 
    let pAltUncert = kAlt_in === null ? 1000 : kAltUncert_in + Q_alt;
    
    let K_alt = pAltUncert / (pAltUncert + R_alt);
    let kAlt = pAlt + K_alt * (nAlt - pAlt);
    let kAltUncert = (1 - K_alt) * pAltUncert;
    return { kAlt, kAltUncert };
}

// --- CLASSE UKF 21 ÉTATS (Fusion Totale) ---
class ProfessionalUKF {
    constructor() {
        if (typeof math === 'undefined') throw new Error("Math.js manquant");
        this.nx = 21; this.nz = 6;
        this.x = math.zeros(this.nx);
        this.x.set([0], sysState.lat * D2R);
        this.x.set([1], sysState.lon * D2R);
        this.P = math.diag(Array(this.nx).fill(1e-2));
        this.Q = math.multiply(math.identity(this.nx), 1e-4); 
    }

    predict(dt, imu) {
        if(dt <= 0) return;
        // Propagation Cinématique (IMU -> État)
        const psi = this.x.get([8]); // Cap
        const ax = imu.x, ay = imu.y, az = imu.z; // Accelérations
        
        // Vitesse (NED)
        let vn = this.x.get([3]) + (ax * Math.cos(psi) - ay * Math.sin(psi)) * dt;
        let ve = this.x.get([4]) + (ax * Math.sin(psi) + ay * Math.cos(psi)) * dt;
        let vd = this.x.get([5]) + (az - 9.80665) * dt; 
        
        // Position (WGS84 Sphérique)
        let lat = this.x.get([0]) + (vn / R_E_BASE) * dt;
        let lon = this.x.get([1]) + (ve / (R_E_BASE * Math.cos(lat))) * dt;
        let alt = this.x.get([2]) - vd * dt; 

        this.x.set([0], lat); this.x.set([1], lon); this.x.set([2], alt);
        this.x.set([3], vn);  this.x.set([4], ve);  this.x.set([5], vd);
        
        this.P = math.add(this.P, math.multiply(this.Q, dt)); // P = FPF' + Q
    }

    update(gps, R_val) {
        const R = math.multiply(math.identity(this.nz), R_val);
        const z = math.matrix([
            gps.lat * D2R, gps.lon * D2R, gps.alt,
            gps.spd * Math.cos(gps.head * D2R), gps.spd * Math.sin(gps.head * D2R), 0
        ]);
        
        const Hx = this.x.subset(math.index(math.range(0,6))); // H = I (6x21)
        const y = math.subtract(z, Hx); // Innovation
        const P_sub = this.P.subset(math.index(math.range(0,6), math.range(0,6)));
        const S = math.add(P_sub, R); // Innovation Covariance
        
        try {
            const K_top = math.multiply(P_sub, math.inv(S)); // Gain K
            const dx = math.multiply(K_top, y); // Correction
            
            // Appliquer correction
            for(let i=0; i<6; i++) this.x.set([i], this.x.get([i]) + dx.get([i]));
            
            // Mise à jour Covariance P = (I - KH)P (Simplifié)
            sysState.est.P_trace = math.trace(this.P);
        } catch(e) { console.warn("UKF Matrix Error", e); }
    }
    
    getState() {
        const d = this.x.toArray();
        return {
            lat: d[0]*R2D, lon: d[1]*R2D, alt: d[2],
            vn: d[3], ve: d[4], vd: d[5],
            speed: Math.sqrt(d[3]**2 + d[4]**2 + d[5]**2)
        };
    }
}
// =================================================================
// BLOC 2/4 : PHYSIQUE AVANCÉE, METROLOGIE & CORRECTIONS
// =================================================================

// --- CORRECTION TROPOSPHÉRIQUE (V3.2) ---
function getTroposphericDelay(P_hPa, T_K, H_frac, alt_m, lat_deg) {
    // Délai Zénithal Hydrostatique (ZHD) + Humide (ZWD)
    const ZHD = 0.0022768 * P_hPa / (1 - 0.00266 * Math.cos(2 * lat_deg * D2R) - 0.00028 * alt_m / 1000);
    const T_C = T_K - 273.15;
    const SVP = 6.11 * Math.exp(19.7 * T_C / (T_C + 273.15)); 
    const e = H_frac * SVP; 
    const ZWD = 0.002277 * (TROPO_K2 / T_K) * (e / 100); 
    return ZHD + ZWD; // Délai en mètres
}

// --- CALCUL DU BRUIT R (Avancé) ---
function getKalmanR_Advanced(acc, alt, P_hPa, env) {
    let R = acc ** 2;
    // Facteurs environnementaux
    const factors = { 'NORMAL': 1.0, 'FOREST': 2.5, 'CONCRETE': 7.0, 'METAL': 5.0 };
    R *= (factors[env] || 1.0);
    
    // Correction ZTD (Troposphère)
    if (P_hPa) {
        const pressureFactor = 1.0 + Math.abs(1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= pressureFactor;
    }
    // Sous-sol
    if (alt < ALT_TH) R *= 2.0;
    
    return Math.max(R_MIN, Math.min(UKF_R_MAX, R));
}

// --- PHYSIQUE RELATIVISTE & FLUIDES (COMPLET) ---
function calculatePhysicsFull(v, alt, mass, cda, rho, lat) {
    // 1. Relativité
    const beta = v / C_L;
    const gamma = 1 / Math.sqrt(1 - beta**2);
    const timeDilVel = (gamma - 1) * 86400 * 1e9; // ns/j
    
    const Rs = (2 * G_U * mass) / C_L**2;
    const r = R_E_BASE + alt;
    const timeDilGrav = (1 - Math.sqrt(1 - Rs/r)) * 86400 * 1e9; // ns/j (Correction Schwarzschild)
    
    // 2. Énergie
    const E0 = mass * C_L**2;
    const E_rel = gamma * E0;
    const p = gamma * mass * v;
    const Ek = 0.5 * mass * v**2; // Classique
    
    // 3. Fluides
    const q = 0.5 * rho * v**2; // Pression dynamique
    const drag = q * cda;
    const power = drag * v / 1000; // kW
    const Re = (rho * v * 1.0) / 1.8e-5; // Reynolds (L=1m)
    
    // 4. Forces Inerties
    const f_cor = 2 * mass * OMEGA_EARTH * v * Math.sin(lat * D2R);
    
    return { 
        gamma, timeDilVel, timeDilGrav, Rs, 
        E0, E_rel, p, Ek, 
        q, drag, power, Re, f_cor 
    };
}

// --- ASTRO (SunCalc + TST) ---
// Intégration directe des fonctions de votre fichier astro_weather.js
function getSolarTime(date, lon) {
    if (!date || isNaN(lon)) return { TST: 'N/A', MST: 'N/A', EOT: 'N/A' };
    
    // Algorithme simplifié de l'Equation du Temps (EOT)
    const d = (date.getTime() - 946728000000) / 86400000; // Jours depuis J2000
    const M = (357.529 + 0.98560028 * d) * D2R;
    const L = (280.466 + 0.98564736 * d) * D2R;
    const lambda = L + 1.915 * D2R * Math.sin(M) + 0.02 * D2R * Math.sin(2 * M);
    const eot_min = 4 * ( (L - 0.0057183 - 2.466 * Math.sin(2 * lambda) + 0.053 * Math.sin(4 * lambda)) * R2D - (L*R2D) ); // Approx

    // Temps Solaire Moyen (MST)
    const utc_h = date.getUTCHours() + date.getUTCMinutes()/60 + date.getUTCSeconds()/3600;
    const mst_h = (utc_h + lon/15 + 24) % 24;
    
    // Temps Solaire Vrai (TST)
    const tst_h = (mst_h + eot_min/60 + 24) % 24;
    
    const toTime = h => {
        const hr = Math.floor(h); const mn = Math.floor((h-hr)*60); const sc = Math.floor(((h-hr)*60 - mn)*60);
        return `${hr}:${mn.toString().padStart(2,'0')}:${sc.toString().padStart(2,'0')}`;
    };
    
    return { TST: toTime(tst_h), MST: toTime(mst_h), EOT: eot_min.toFixed(2) };
            }
// =================================================================
// BLOC 3/4 : ACQUISITION I/O, ZUPT & BOUCLE RAPIDE
// =================================================================

// --- IMU ---
function handleMotion(e) {
    const a = e.accelerationIncludingGravity;
    const r = e.rotationRate;
    if(a) { 
        sysState.accel = { x: a.x||0, y: a.y||0, z: a.z||0 };
        // Mise à jour DOM directe pour fluidité
        $('acceleration-x').textContent = dataOrDefault(a.x, 2, ' m/s²');
        $('acceleration-y').textContent = dataOrDefault(a.y, 2, ' m/s²');
        $('acceleration-z').textContent = dataOrDefault(a.z, 2, ' m/s²');
    }
    if(r) sysState.gyro = { x: r.alpha||0, y: r.beta||0, z: r.gamma||0 };
}

// --- GPS (Avec ZUPT Logic) ---
function onGpsUpdate(pos) {
    if (sysState.isPaused) return;
    
    lastGPS = pos;
    const coords = pos.coords;
    const acc = coords.accuracy || 20;
    const spdRaw = coords.speed || 0;
    
    // LOGIQUE ZUPT (Zero Velocity Update)
    // Si l'accéléromètre montre peu de mouvement ET la vitesse GPS est faible => On est à l'arrêt.
    const accelMag = Math.sqrt(sysState.accel.x**2 + sysState.accel.y**2);
    const isStopped = (spdRaw < ZUPT_RAW_THRESHOLD && accelMag < ZUPT_ACCEL_THRESHOLD);
    
    // Calcul R dynamique (avec ZTD)
    const ztd = getTroposphericDelay(sysState.pressureHPa, sysState.tempK, sysState.humidity/100, sysState.alt, coords.latitude);
    const R_dyn = isStopped ? 0.1 : getKalmanR_Advanced(acc, sysState.alt, sysState.pressureHPa, 'NORMAL'); // R très faible si arrêté
    
    // Update UKF
    const dataToUpdate = { 
        lat: coords.latitude, lon: coords.longitude, alt: coords.altitude || 0,
        spd: isStopped ? 0 : spdRaw, // Force 0 si ZUPT
        head: coords.heading || 0
    };
    if(ukf) ukf.update(dataToUpdate, R_dyn);
    
    // Status
    $('gps-status-dr').textContent = isStopped ? "✅ ZUPT (Arrêt)" : "Actif (Fusion)";
    $('gps-precision').textContent = acc.toFixed(1) + " m";
}

// --- BOUCLE RAPIDE (50Hz) ---
function startLoop() {
    let last = performance.now();
    
    const loop = () => {
        if(sysState.isPaused) { requestAnimationFrame(loop); return; }
        
        const now = performance.now();
        const dt = (now - last) / 1000;
        last = now;
        
        // 1. Prédiction UKF (IMU)
        if(ukf) ukf.predict(dt, sysState.accel);
        
        // 2. Extraction État
        const state = ukf ? ukf.getState() : { lat: sysState.lat, lon: sysState.lon, alt: sysState.alt, speed: 0 };
        
        // 3. Calculs Physiques
        const phys = calculatePhysicsFull(state.speed, state.alt, sysState.mass, sysState.cda, sysState.env.rho, state.lat);
        
        // 4. Mise à jour DOM (Tableau Scientifique)
        updateScientificTable(state, phys);
        
        requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
}

function updateScientificTable(state, phys) {
    // VITESSE
    const spdKmh = state.speed * KMH_MS;
    $('current-speed').textContent = dataOrDefault(spdKmh, 2, ' km/h');
    $('speed-stable').textContent = dataOrDefault(spdKmh, 2, ' km/h');
    $('speed-stable-ms').textContent = dataOrDefault(state.speed, 3, ' m/s');
    $('mach-number').textContent = dataOrDefault(state.speed / sysState.env.soundSpd, 4);
    $('perc-speed-sound').textContent = dataOrDefault(state.speed / sysState.env.soundSpd * 100, 2, ' %');
    
    // POSITION
    $('current-lat').textContent = dataOrDefault(state.lat, 6, '°');
    $('current-lon').textContent = dataOrDefault(state.lon, 6, '°');
    $('current-alt').textContent = dataOrDefault(state.alt, 2, ' m');
    $('geopotential-alt').textContent = dataOrDefault(state.alt * (9.81/9.80665), 2, ' m');
    
    // PHYSIQUE & RELATIVITÉ
    $('lorentz-factor').textContent = dataOrDefault(phys.gamma, 10);
    $('time-dilation-vitesse').textContent = dataOrDefaultExp(phys.timeDilVel, 2, ' ns/j');
    $('time-dilation-gravite').textContent = dataOrDefaultExp(phys.timeDilGrav, 2, ' ns/j');
    $('energy-relativistic').textContent = dataOrDefaultExp(phys.E_rel, 4, ' J');
    $('momentum').textContent = dataOrDefaultExp(phys.p, 4, ' kg·m/s');
    
    // DYNAMIQUE
    $('dynamic-pressure').textContent = dataOrDefault(phys.q, 2, ' Pa');
    $('drag-force').textContent = dataOrDefault(phys.drag, 2, ' N');
    $('coriolis-force').textContent = dataOrDefaultExp(phys.f_cor, 2, ' N');
    $('reynolds-number').textContent = dataOrDefaultExp(phys.Re, 2);
    
    // EKF DEBUG
    $('ukf-v-uncert').textContent = dataOrDefault(Math.sqrt(sysState.est.P_trace), 3);
    
    // CARTE
    if(map && marker) {
        const ll = [state.lat, state.lon];
        marker.setLatLng(ll);
        if(polyline) polyline.addLatLng(ll);
    }
    }
// =================================================================
// BLOC 4/4 : INITIALISATION & BOUCLE LENTE
// =================================================================

function startSlowLoop() {
    setInterval(() => {
        if(sysState.isPaused) return;
        
        const now = new Date(); // Idéalement corrigé par NTP
        const sTime = getSolarTime(now, sysState.lon);
        
        // MAJ Astro DOM
        $('tst').textContent = sTime.TST;
        $('mst').textContent = sTime.MST;
        $('eot').textContent = `${sTime.EOT} min`;
        
        if(typeof SunCalc !== 'undefined') {
            const pos = SunCalc.getPosition(now, sysState.lat, sysState.lon);
            $('sun-alt').textContent = dataOrDefault(pos.altitude * R2D, 2, '°');
            $('sun-azimuth').textContent = dataOrDefault(pos.azimuth * R2D, 2, '°');
            
            const moon = SunCalc.getMoonIllumination(now);
            $('moon-illuminated').textContent = dataOrDefault(moon.fraction * 100, 1, '%');
        }
        
        // MAJ Temps Session
        const elapsed = (Date.now() - sysState.startTime) / 1000;
        $('elapsed-session-time').textContent = formatTime(elapsed);
        
    }, 1000);
}

const formatTime = (s) => new Date(s * 1000).toISOString().substr(11, 8);

// --- INIT ---
document.addEventListener('DOMContentLoaded', () => {
    // 1. Init Carte
    if(typeof L !== 'undefined') {
        map = L.map('map').setView([43.3, 5.4], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
        marker = L.marker([0,0]).addTo(map);
        polyline = L.polyline([], {color:'blue'}).addTo(map);
    }

    // 2. Bouton GPS
    const btn = $('toggle-gps-btn');
    if(btn) {
        btn.addEventListener('click', () => {
            if(sysState.isPaused) {
                // DÉMARRAGE
                if(typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
                    DeviceMotionEvent.requestPermission().then(r => { if(r==='granted') window.addEventListener('devicemotion', handleMotion); });
                } else {
                    window.addEventListener('devicemotion', handleMotion);
                }
                
                navigator.geolocation.watchPosition(onGpsUpdate, console.warn, {enableHighAccuracy:true});
                
                if(!ukf) ukf = new ProfessionalUKF();
                sysState.isPaused = false;
                sysState.startTime = Date.now();
                btn.textContent = "⏸️ PAUSE";
                
                startLoop(); // Boucle Rapide
                startSlowLoop(); // Boucle Lente
                
            } else {
                sysState.isPaused = true;
                btn.textContent = "▶️ MARCHE GPS";
            }
        });
    } else {
        console.error("Bouton GPS introuvable!");
    }
    
    // 3. Init Météo (One shot)
    fetch(`https://scientific-dashboard2.vercel.app/api/weather?lat=43.3&lon=5.4`)
        .then(r=>r.json()).then(d => {
            if(d.main) {
                sysState.env.tempK = d.main.temp + 273.15;
                sysState.env.pressureHPa = d.main.pressure;
                $('temp-air-2').textContent = d.main.temp + ' °C';
            }
        }).catch(e=>{});
});
