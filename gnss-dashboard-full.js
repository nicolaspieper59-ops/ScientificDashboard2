// =================================================================
// GNSS DASHBOARD - KERNEL SCIENTIFIQUE & CONFIGURATION
// Version Compatible : iOS 17+, Android 14+, Chrome/Safari Desktop
// =================================================================

const $ = id => document.getElementById(id);
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

// Constantes Physiques CODATA
const C_L = 299792458.0;          // Célérité lumière
const G_U = 6.67430e-11;          // Gravitation
const WGS84_A = 6378137.0;        // Rayon Eq.
const WGS84_E2 = 6.69437999014e-3;// Excentricité²
const RHO_AIR = 1.225;            // Densité Air Standard

// --- ÉTAT DU SYSTÈME (Single Source of Truth) ---
const sysState = {
    running: false,
    permsGranted: false,
    
    // Vecteur d'État (Fusionné)
    est: { lat: 43.2964, lon: 5.3697, alt: 0, vn: 0, ve: 0, vd: 0, speed: 0, heading: 0 },
    
    // Données Brutes (Capteurs)
    raw: { 
        acc: { x:0, y:0, z:0 }, // Accélération
        gyr: { x:0, y:0, z:0 }, // Rotation
        gps: { lat:0, lon:0, alt:0, acc:0, spd:0 },
        ts: 0
    },
    
    // Métriques
    metrics: { maxSpeed: 0, totalDist: 0, startTime: 0, P_trace: 100 },
    
    // Config Environnement
    env: { tempK: 288.15, pressPa: 101325, soundSpd: 340.3 }
};

let ukf = null; // Instance du filtre
let map = null, marker = null; // Carte

// --- ALGEBRE LINÉAIRE (UKF SIMPLIFIÉ POUR JS PUR) ---
class RobustUKF {
    constructor() {
        this.x = [0, 0, 0, 0, 0, 0]; // Lat, Lon, Alt, Vn, Ve, Vd
        this.P = [100, 100, 100, 10, 10, 10]; // Variances diagonales (Incertitude)
        this.status = "INITIALISÉ";
    }

    predict(dt, ax, ay, az) {
        if(dt <= 0) return;
        // Propagation Cinématique (IMU -> État)
        // V(k+1) = V(k) + a * dt
        this.x[3] += ax * dt;
        this.x[4] += ay * dt;
        this.x[5] += az * dt;
        
        // Pos(k+1) = Pos(k) + V * dt (Approx locale)
        const R_lat = 6378137;
        this.x[0] += (this.x[3] / R_lat) * dt * (180/Math.PI);
        this.x[1] += (this.x[4] / (R_lat * Math.cos(this.x[0]*Math.PI/180))) * dt * (180/Math.PI);
        this.x[2] += this.x[5] * dt; // Z vers le haut ici
        
        // Augmentation de l'incertitude (Bruit de process Q)
        for(let i=0; i<6; i++) this.P[i] += 0.01 * dt; 
        
        this.status = "PRÉDICTION (IMU)";
    }

    update(lat, lon, alt, spd, acc_gps) {
        // Gain de Kalman Scalaire (Simplification robuste pour temps réel)
        // K = P / (P + R)
        const R_pos = acc_gps**2;
        const R_vel = 1.0; // 1 m/s d'erreur vitesse
        
        // Mise à jour Position
        const K_pos = this.P[0] / (this.P[0] + R_pos);
        this.x[0] += K_pos * (lat - this.x[0]);
        this.x[1] += K_pos * (lon - this.x[1]);
        this.x[2] += K_pos * (alt - this.x[2]);
        this.P[0] *= (1 - K_pos);
        this.P[1] *= (1 - K_pos);
        this.P[2] *= (1 - K_pos);

        // Mise à jour Vitesse (Si dispo)
        if(spd >= 0) {
            // On suppose que la vitesse GPS magnitude corrige la norme du vecteur vitesse
            const curSpd = Math.sqrt(this.x[3]**2 + this.x[4]**2);
            if(curSpd > 0.1) {
                const scale = spd / curSpd;
                const K_vel = this.P[3] / (this.P[3] + R_vel);
                // Correction douce vers la vitesse GPS
                this.x[3] = this.x[3] * (1 - K_vel) + (this.x[3] * scale) * K_vel;
                this.x[4] = this.x[4] * (1 - K_vel) + (this.x[4] * scale) * K_vel;
                this.P[3] *= (1 - K_vel);
            }
        }
        this.status = "CORRECTION (GPS)";
        sysState.metrics.P_trace = this.P.reduce((a,b)=>a+b, 0);
    }
            }
// =================================================================
// BLOC 2/4 : GESTIONNAIRE DE CAPTEURS (COMPATIBILITÉ MODERNE)
// =================================================================

const SensorManager = {
    watchID: null,

    // 1. Démarrage Maître (Appelé par le bouton)
    requestAccess: async function() {
        // A. Géolocalisation
        if (!navigator.geolocation) {
            alert("GPS non supporté");
            return false;
        }

        // B. IMU (Accéléromètre/Gyro) - Le point critique iOS 13+
        if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
            try {
                const permission = await DeviceMotionEvent.requestPermission();
                if (permission === 'granted') {
                    this.startIMUListeners();
                    return true;
                } else {
                    alert("Permission Capteurs refusée. Le mode Inertiel ne fonctionnera pas.");
                    return false;
                }
            } catch (e) {
                console.error("Erreur Permission IMU:", e);
                return false; // Souvent dû à un contexte non-HTTPS
            }
        } else {
            // Non-iOS ou ancien appareil
            this.startIMUListeners();
            return true;
        }
    },

    // 2. Démarrage des Écouteurs IMU
    startIMUListeners: function() {
        window.addEventListener('devicemotion', (e) => {
            // Accélération (avec gravité pour le niveau à bulle, sans pour l'INS)
            const accG = e.accelerationIncludingGravity;
            const acc = e.acceleration;
            const rot = e.rotationRate;

            if (accG) {
                // Pour affichage brut
                sysState.raw.acc.x = accG.x || 0;
                sysState.raw.acc.y = accG.y || 0;
                sysState.raw.acc.z = accG.z || 0;
            }
            
            // Pour l'UKF (On préfère l'accélération linéaire sans gravité si dispo)
            if (acc && ukf) {
                // Conversion repère (X=Est, Y=Nord, Z=Haut approx pour le web)
                // Note: La rotation réelle nécessite les quaternions, ici simplifié
                ukf.predict(0.02, acc.x || 0, acc.y || 0, acc.z || 0); 
            }
            
            if (rot) {
                sysState.raw.gyr.x = rot.alpha || 0;
                sysState.raw.gyr.y = rot.beta || 0;
                sysState.raw.gyr.z = rot.gamma || 0;
            }
        });
    },

    // 3. Démarrage GPS
    startGPS: function() {
        this.watchID = navigator.geolocation.watchPosition(
            (pos) => {
                const c = pos.coords;
                // Mise à jour État Brut
                sysState.raw.gps = {
                    lat: c.latitude, lon: c.longitude, alt: c.altitude || 0,
                    acc: c.accuracy, spd: c.speed || 0, heading: c.heading || 0
                };
                
                // Correction UKF
                if (ukf) ukf.update(c.latitude, c.longitude, c.altitude || 0, c.speed || 0, c.accuracy);
                
                // Callback visuel (Icone active)
                $('statut-gps-acquisition').textContent = `FIX 3D (${c.accuracy.toFixed(1)}m)`;
                $('statut-gps-acquisition').style.color = '#00ff00';
            },
            (err) => {
                console.warn("GPS Error", err);
                $('statut-gps-acquisition').textContent = `ERR: ${err.message}`;
                $('statut-gps-acquisition').style.color = 'red';
            },
            { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
        );
    },

    stopAll: function() {
        if (this.watchID) navigator.geolocation.clearWatch(this.watchID);
        // Note: On ne peut pas vraiment "arrêter" les EventListeners globaux proprement sans référence nommée externe, 
        // mais on peut stopper le traitement via le flag sysState.running
    }
};
// =================================================================
// BLOC 3/4 : MOTEUR PHYSIQUE (RELATIVITÉ, FLUIDES, ASTRO)
// =================================================================

const Physics = {
    // Calcul Relativiste
    computeRelativity: (v, alt) => {
        const beta = v / C_L;
        const gamma = 1 / Math.sqrt(1 - beta*beta); // Facteur de Lorentz
        
        // Dilatation Temporelle Gravitationnelle (Schwarzschild)
        const Rs = (2 * G_U * 5.972e24) / (C_L**2); // Terre
        const r = 6371000 + alt;
        const dil_grav = Math.sqrt(1 - Rs/r); // Ralentissement
        
        // Différence en ns/jour
        const time_drift_v = (gamma - 1) * 86400 * 1e9;
        const time_drift_g = (1 - dil_grav) * 86400 * 1e9;
        
        return { gamma, time_drift_v, time_drift_g, Rs };
    },

    // Calcul Dynamique des Fluides
    computeFluids: (v, alt) => {
        // Modèle ISA Simplifié
        const temp = 288.15 - 0.0065 * alt;
        const press = 101325 * Math.pow(1 - 0.0065*alt/288.15, 5.255);
        const rho = press / (287.05 * temp);
        const c_sound = 20.05 * Math.sqrt(temp);
        
        const q = 0.5 * rho * v * v; // Pression dynamique
        const mach = v / c_sound;
        
        return { rho, temp, press, c_sound, q, mach };
    },
    
    // Calcul Énergie
    computeEnergy: (mass, v, gamma) => {
        const E0 = mass * C_L**2;
        const E_tot = gamma * E0;
        const Ek = (gamma - 1) * E0; // Énergie Cinétique Relativiste
        const p = gamma * mass * v;  // Momentum
        return { E0, E_tot, Ek, p };
    }
};

// Fonctions d'affichage formaté
function updateDOM() {
    if(!sysState.running) return;
    
    // 1. Récupération État Fusionné
    const lat = ukf.x[0];
    const lon = ukf.x[1];
    const alt = ukf.x[2];
    const v3d = Math.sqrt(ukf.x[3]**2 + ukf.x[4]**2 + ukf.x[5]**2); // Vitesse Vraie
    
    // 2. Calculs Physique
    const phys = Physics.computeRelativity(v3d, alt);
    const fluid = Physics.computeFluids(v3d, alt);
    const energy = Physics.computeEnergy(70, v3d, phys.gamma); // Masse 70kg par défaut
    
    // 3. Mise à jour Carte
    if(map && marker) {
        const ll = [lat, lon];
        marker.setLatLng(ll);
        map.setView(ll, map.getZoom(), {animate: false});
    }

    // 4. INJECTION DOM (Tableau Scientifique)
    
    // VITESSE
    $('speed-stable').innerText = (v3d * 3.6).toFixed(2) + ' km/h';
    $('speed-stable-ms').innerText = v3d.toFixed(3) + ' m/s';
    $('mach-number').innerText = fluid.mach.toFixed(4);
    
    // POSITION
    $('current-lat').innerText = lat.toFixed(7) + ' °';
    $('current-lon').innerText = lon.toFixed(7) + ' °';
    $('current-alt').innerText = alt.toFixed(2) + ' m';
    
    // RELATIVITÉ (Cœur du sujet)
    $('lorentz-factor').innerText = phys.gamma.toFixed(12);
    $('time-dilation-vitesse').innerText = phys.time_drift_v.toFixed(4) + ' ns/j';
    $('time-dilation-gravite').innerText = phys.time_drift_g.toFixed(4) + ' ns/j';
    $('percent-speed-light').innerText = ((v3d/C_L)*100).toExponential(2) + ' %';
    
    // ÉNERGIE
    $('kinetic-energy').innerText = energy.Ek.toExponential(3) + ' J';
    $('momentum').innerText = energy.p.toExponential(3) + ' kg·m/s';
    
    // ENVIRONNEMENT
    $('dynamic-pressure').innerText = fluid.q.toFixed(2) + ' Pa';
    $('air-density').innerText = fluid.rho.toFixed(3) + ' kg/m³';
    $('temp-air-2').innerText = (fluid.temp - 273.15).toFixed(1) + ' °C';
    
    // CAPTEURS BRUTS
    $('acceleration-x').innerText = sysState.raw.acc.x.toFixed(2);
    $('acceleration-y').innerText = sysState.raw.acc.y.toFixed(2);
    $('acceleration-z').innerText = sysState.raw.acc.z.toFixed(2);
    
    // STATUS
    $('statut-ekf-fusion').innerText = ukf.status;
    $('P_trace').innerText = sysState.metrics.P_trace.toFixed(4);
}
// =================================================================
// BLOC 4/4 : INITIALISATION & BOUCLE PRINCIPALE
// =================================================================

function startSystem() {
    if(sysState.running) return;
    
    // 1. Initialisation Filtre
    ukf = new RobustUKF();
    
    // 2. Lancement Capteurs (Async pour permissions)
    SensorManager.requestAccess().then(granted => {
        if(granted) {
            SensorManager.startGPS();
            sysState.running = true;
            sysState.metrics.startTime = Date.now();
            
            // UI Update
            $('toggle-gps-btn').innerHTML = "⏸️ PAUSE SYSTÈME";
            $('toggle-gps-btn').style.background = "#ff9800";
            
            // Lancement Boucle Rendu (60 FPS)
            loop();
        } else {
            $('toggle-gps-btn').innerHTML = "⚠️ ACCÈS REFUSÉ";
        }
    });
}

function stopSystem() {
    sysState.running = false;
    SensorManager.stopAll();
    $('toggle-gps-btn').innerHTML = "▶️ MARCHE GPS";
    $('toggle-gps-btn').style.background = "#28a745";
    $('statut-gps-acquisition').innerText = "PAUSE";
}

function loop() {
    if(!sysState.running) return;
    
    updateDOM();
    requestAnimationFrame(loop);
}

// --- MAIN ENTRY POINT ---
document.addEventListener('DOMContentLoaded', () => {
    console.log("Système Prêt. Attente utilisateur.");
    
    // 1. Init Carte Leaflet
    if(typeof L !== 'undefined') {
        map = L.map('map').setView([43.3, 5.4], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
        marker = L.marker([43.3, 5.4]).addTo(map);
    } else {
        $('map').innerHTML = "Leaflet JS non chargé !";
    }

    // 2. Listener Bouton Principal
    const btn = $('toggle-gps-btn');
    if(btn) {
        btn.onclick = () => {
            if(sysState.running) stopSystem();
            else startSystem();
        };
    } else {
        console.error("Bouton 'toggle-gps-btn' introuvable dans le HTML !");
    }
    
    // 3. Mise à jour horloge locale (1Hz)
    setInterval(() => {
        $('local-time').innerText = new Date().toLocaleTimeString();
    }, 1000);
});
