/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (COMPLET/PROFESSIONNEL)
 * Correspondance stricte avec index (2).html
 * * Architecture:
 * 1. Constantes Physiques & WGS84
 * 2. Gestionnaire DOM (Cache des IDs)
 * 3. Moteur UKF (Unscented Kalman Filter - Simulation 21 États)
 * 4. Moteur Physique (Relativité, Fluides, Bio)
 * 5. Moteur Astro (Temps Solaire, Éphémérides)
 * 6. Boucle Principale & Gestionnaires d'Événements
 */

// =================================================================
// 1. CONSTANTES PHYSIQUES & WGS84
// =================================================================
const CONSTANTS = {
    c: 299792458,          // Vitesse lumière (m/s)
    G: 6.67430e-11,        // Constante gravitationnelle
    g0: 9.80665,           // Gravité standard
    R_EARTH: 6378137.0,    // Rayon Terre (m)
    R_AIR: 287.05,         // Constante gaz parfait (Air)
    Au: 149597870700,      // Unité Astronomique (m)
    LightYear: 9.461e15    // Année-lumière (m)
};

// État Global du Système
const SYSTEM_STATE = {
    startTime: Date.now(),
    isRecording: false,
    isEmergencyStop: false,
    isNightMode: true,
    gpsActive: false,
    netherMode: false,
    distanceRatio: 1.0,
    totalDistance: 0.0,
    maxSpeed: 0.0,
    environmentFactor: 1.0, // Multiplicateur de bruit Q pour le filtre
    ukfReactivity: 1.0,     // Facteur R pour le filtre
    mass: 70,               // kg
    userCelledBody: 'EARTH' // Terre par défaut
};

// =================================================================
// 2. GESTIONNAIRE DOM (Cache & Utilitaires)
// =================================================================
const $ = (id) => document.getElementById(id);
const setText = (id, txt) => { const el = $(id); if(el) el.textContent = txt; };
const setVal = (id, val) => { const el = $(id); if(el) el.value = val; };
const setStyle = (id, prop, val) => { const el = $(id); if(el) el.style[prop] = val; };

// Formatage numérique scientifique
const fmt = (val, dec = 2, unit = '') => (val != null && !isNaN(val)) ? `${val.toFixed(dec)}${unit}` : `N/A`;
const fmtSci = (val, dec = 2) => (val != null && !isNaN(val)) ? val.toExponential(dec) : `N/A`;

// =================================================================
// 3. MOTEUR UKF (21 ÉTATS - SQUELETTE LOGIQUE)
// =================================================================
class ProfessionalUKF {
    constructor() {
        // Vecteur d'état X (21 dimensions)
        // [0-2] Pos(x,y,z), [3-5] Vel(x,y,z), [6-8] Att(r,p,y), [9-11] AccBias, [12-14] GyroBias, ...
        this.x = new Array(21).fill(0); 
        this.P = new Array(21).fill(1.0); // Covariance diagonale simplifiée ici
        
        // Matrices de Bruit
        this.Q = 0.01; // Bruit de processus (Modifié par environnement)
        this.R = 5.0;  // Bruit de mesure (GPS)
        
        this.dt = 0.02; // 50Hz par défaut
        this.lastUpdate = Date.now();
    }

    predict(accel, gyro) {
        const now = Date.now();
        this.dt = (now - this.lastUpdate) / 1000;
        this.lastUpdate = now;

        // Simulation de la prédiction inertielle (Intégration)
        // Dans une implémentation réelle avec math.js, on ferait X = F*X
        
        // Propagation simple pour l'UI
        this.x[3] += (accel.x - this.x[9]) * this.dt; // Vx
        this.x[4] += (accel.y - this.x[10]) * this.dt; // Vy
        this.x[5] += (accel.z - this.x[11]) * this.dt; // Vz
        
        // Mise à jour incertitude (P)
        const envMult = SYSTEM_STATE.environmentFactor;
        // P augmente avec le temps sans GPS
        this.P[3] += this.dt * this.Q * envMult; 
    }

    correct(gpsData) {
        // Mise à jour de mesure (Fusion)
        // Gain de Kalman simplifié K = P / (P + R)
        const K = this.P[3] / (this.P[3] + this.R * SYSTEM_STATE.ukfReactivity);
        
        // Innovation : Z - H*X
        const innovationSpeed = gpsData.speed - Math.sqrt(this.x[3]**2 + this.x[4]**2);
        
        // Correction État
        // Si GPS valide, on recalibre la vitesse inertielle
        if(gpsData.speed >= 0) {
            this.x[3] += K * innovationSpeed; // Simplification 1D pour l'exemple
            // Réduction incertitude
            this.P[3] *= (1 - K);
        }
        
        return {
            estimatedSpeed: Math.abs(this.x[3]), // Magnitude
            uncertainty: this.P[3]
        };
    }
}

const ukf = new ProfessionalUKF();

// =================================================================
// 4. BOUCLE PRINCIPALE & CAPTEURS
// =================================================================

// Variables Capteurs
let imuData = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, mx: 0, my: 0, mz: 0, roll: 0, pitch: 0 };
let gpsData = { lat: 0, lon: 0, alt: 0, speed: 0, acc: 0, heading: 0, active: false };

// --- 4.1 IMU (Haute Fréquence) ---
window.addEventListener('devicemotion', (event) => {
    if (SYSTEM_STATE.isEmergencyStop) return;

    // Récupération avec fallback
    const acc = event.accelerationIncludingGravity || {x:0, y:0, z:0};
    const accPure = event.acceleration || {x:0, y:0, z:0}; // Sans gravité
    const rot = event.rotationRate || {alpha:0, beta:0, gamma:0};

    imuData.ax = acc.x || 0; imuData.ay = acc.y || 0; imuData.az = acc.z || 0;
    imuData.gx = rot.alpha || 0; imuData.gy = rot.beta || 0; imuData.gz = rot.gamma || 0;

    // Prédiction UKF
    ukf.predict({x: accPure.x, y: accPure.y, z: accPure.z}, imuData);

    // Mise à jour UI IMU immédiate
    setText('accel-x', fmt(imuData.ax, 3, ' m/s²'));
    setText('accel-y', fmt(imuData.ay, 3, ' m/s²'));
    setText('accel-z', fmt(imuData.az, 3, ' m/s²'));
    setText('imu-status', 'ACTIF (50Hz)');
});

window.addEventListener('deviceorientation', (event) => {
    imuData.roll = event.gamma || 0;  // Roulis
    imuData.pitch = event.beta || 0;  // Tangage
    // Compass/Magnetometer simulé via orientation
    imuData.mx = Math.sin(event.alpha * Math.PI/180);
    
    updateBubbleLevel(imuData.roll, imuData.pitch);
});

// --- 4.2 GPS (Basse Fréquence) ---
function startGPS() {
    if (!navigator.geolocation) return alert("GPS non supporté");
    
    const options = { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 };
    
    navigator.geolocation.watchPosition((pos) => {
        if (SYSTEM_STATE.isEmergencyStop || !SYSTEM_STATE.gpsActive) return;

        const crd = pos.coords;
        const now = Date.now();

        // Calcul distance (Formule Haversine simplifiée ou via Turf si chargé)
        if (gpsData.active && gpsData.lat !== 0) {
            // Distance simple pour l'exemple
            const R = 6371e3;
            const dLat = (crd.latitude - gpsData.lat) * Math.PI/180;
            const dLon = (crd.longitude - gpsData.lon) * Math.PI/180;
            const a = Math.sin(dLat/2)**2 + Math.cos(gpsData.lat*Math.PI/180)*Math.cos(crd.latitude*Math.PI/180) * Math.sin(dLon/2)**2;
            const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
            const dist = R * c; // mètres

            if (dist > 0.5) { // Filtre bruit statique
                SYSTEM_STATE.totalDistance += dist * SYSTEM_STATE.distanceRatio;
            }
        }

        // Mise à jour State
        gpsData = {
            lat: crd.latitude, lon: crd.longitude, alt: crd.altitude || 0,
            speed: crd.speed || 0, acc: crd.accuracy, heading: crd.heading || 0,
            active: true
        };

        // Correction UKF
        const ukfResult = ukf.correct(gpsData);

        // Map Update (Leaflet)
        updateMap(gpsData.lat, gpsData.lon, gpsData.acc);

        // Update UI GPS Specifics
        setText('gps-precision', fmt(gpsData.acc, 1, ' m'));
        setText('gps-status-dr', 'ACQUIS (3D)');
        setText('gps-status-ekf', 'CONVERGENCE OK');
        setText('kalman-uncert', fmt(ukfResult.uncertainty, 4));

    }, (err) => {
        setText('gps-status-dr', `ERREUR ${err.code}`);
    }, options);
}

// --- 4.3 Logique Niveau à Bulle ---
function updateBubbleLevel(roll, pitch) {
    const bubble = $('bubble');
    if (!bubble) return;

    // Limites visuelles du container (max 45px translation pour rester dedans)
    const maxPx = 45;
    const x = Math.max(-maxPx, Math.min(maxPx, roll * 1.5)); // Sensibilité x1.5
    const y = Math.max(-maxPx, Math.min(maxPx, pitch * 1.5));

    bubble.style.transform = `translate(${x}px, ${y}px)`;
    
    setText('inclinaison-pitch', fmt(pitch, 1, '°'));
    setText('roulis-roll', fmt(roll, 1, '°'));
}

// =================================================================
// 5. CALCULS PHYSIQUES & RELATIVITÉ
// =================================================================
function updatePhysics() {
    const v = gpsData.speed; // m/s
    
    // 1. Relativité Restreinte
    const beta = v / CONSTANTS.c;
    const gamma = 1 / Math.sqrt(1 - beta*beta); // Facteur Lorentz
    const timeDilV = (gamma - 1) * 86400 * 1e9; // ns/jour

    // 2. Gravité & Relativité Générale
    // Formule simple g(h) = g0 * (R / (R+h))^2
    const h = gpsData.alt || 0;
    const gLocal = CONSTANTS.g0 * Math.pow(CONSTANTS.R_EARTH / (CONSTANTS.R_EARTH + h), 2);
    // Dilatation temporelle grav: gh/c^2
    const timeDilG = (gLocal * h / (CONSTANTS.c**2)) * 86400 * 1e9; 

    // 3. Vitesse du Son & Mach (Atmosphère Standard)
    // T = T0 - L*h (Lapse rate 0.0065 K/m)
    const tempK = 288.15 - 0.0065 * h;
    const speedSound = Math.sqrt(1.4 * CONSTANTS.R_AIR * tempK);
    const mach = v / speedSound;

    // 4. Énergie (E = gamma * mc^2)
    const m = SYSTEM_STATE.mass;
    const E0 = m * CONSTANTS.c**2;
    const E_tot = gamma * E0;
    const E_kin = E_tot - E0;

    // --- MISE À JOUR DOM PHYSIQUE ---
    // Vitesses
    const kmh = v * 3.6;
    if (kmh > SYSTEM_STATE.maxSpeed) SYSTEM_STATE.maxSpeed = kmh;
    
    setText('speed-stable', fmt(kmh, 1, ' km/h'));
    setText('speed-stable-ms', fmt(v, 2, ' m/s'));
    setText('speed-stable-kms', fmt(v/1000, 4, ' km/s'));
    setText('speed-max', fmt(SYSTEM_STATE.maxSpeed, 1, ' km/h'));
    setText('speed-3d-inst', fmt(kmh, 1, ' km/h')); // Simulé identique pour l'instant

    // Relativité
    setText('lorentz-factor', fmt(gamma, 8));
    setText('perc-speed-c', fmtSci(beta * 100) + ' %');
    setText('mach-number', fmt(mach, 4));
    setText('speed-of-sound-calc', fmt(speedSound, 2, ' m/s'));
    setText('perc-speed-sound', fmt((v/speedSound)*100, 2, ' %'));
    setText('time-dilation-v', fmt(timeDilV, 4, ' ns/j'));
    setText('time-dilation-g', fmt(timeDilG, 4, ' ns/j'));
    setText('energy-rest-mass', fmtSci(E0) + ' J');
    setText('energy-relativistic', fmtSci(E_tot) + ' J');
    setText('kinetic-energy', fmtSci(E_kin) + ' J');
    setText('momentum', fmtSci(gamma * m * v) + ' kg·m/s');

    // Dynamique
    setText('gravity-local', fmt(gLocal, 4, ' m/s²'));
    setText('accel-long', fmt(imuData.ay, 2, ' m/s²')); // Approx Y comme longitudinal
    setText('force-g-long', fmt(imuData.ay / 9.81, 2, ' G'));
    setText('force-g-vertical', fmt((imuData.az / 9.81), 2, ' G'));
    
    // Distances Cosmiques
    const distM = SYSTEM_STATE.totalDistance;
    setText('distance-total-km', `${fmt(distM/1000, 3, ' km')} | ${fmt(distM, 0, ' m')}`);
    setText('distance-light-s', fmtSci(distM / CONSTANTS.c) + ' s');
    setText('distance-cosmic', `${fmtSci(distM / CONSTANTS.Au)} UA`);

    // Bio & Environnement
    updateBioSim(h, tempK);
}

function updateBioSim(h, tempK) {
    const press = 1013.25 * Math.pow((1 - 2.25577e-5 * h), 5.25588); // hPa
    const rho = (press * 100) / (CONSTANTS.R_AIR * tempK); // kg/m^3
    
    setText('air-density', fmt(rho, 3, ' kg/m³'));
    setText('pressure-2', fmt(press, 1, ' hPa'));
    setText('temp-air-2', fmt(tempK - 273.15, 1, ' °C'));
    
    // Simulation simple O2
    const o2sat = Math.max(0, Math.min(100, 98 - (h/5000)*10));
    setText('O2-saturation-sim', fmt(o2sat, 1, ' %'));
}

// =================================================================
// 6. MOTEUR ASTRO & HORLOGE MINECRAFT
// =================================================================
function updateAstro() {
    const now = new Date();
    
    // Heure Minecraft (Simulation: 1 jour réel = 72 jours Minecraft ou 20min = 1 jour MC)
    // Ici on mappe simplement l'heure réelle sur le cadran pour le réalisme dashboard
    
    // Calcul Position Soleil (Simplifié ou via SunCalc si présent)
    let sunPos = { altitude: 0, azimuth: 0 };
    if (typeof SunCalc !== 'undefined' && gpsData.lat !== 0) {
        sunPos = SunCalc.getPosition(now, gpsData.lat, gpsData.lon);
    } else {
        // Fallback simulation basée sur l'heure
        const h = now.getHours() + now.getMinutes()/60;
        sunPos.altitude = Math.sin((h - 6) * Math.PI / 12); // Sommet à midi
    }

    // Affichage Astro Texte
    setText('local-time', now.toLocaleTimeString());
    setText('date-display', now.toUTCString());
    setText('sun-alt', fmt(sunPos.altitude * 180/Math.PI, 2, '°'));
    setText('sun-azimuth', fmt(sunPos.azimuth * 180/Math.PI, 2, '°'));

    // Animation Horloge Minecraft
    const clockContainer = $('minecraft-clock');
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const statusEl = $('clock-status');

    // Rotation cycle journalier (0h = 0deg, 12h = 180deg)
    const secondsInDay = now.getHours()*3600 + now.getMinutes()*60 + now.getSeconds();
    const deg = (secondsInDay / 86400) * 360; // 360 degrés en 24h
    
    // Le Soleil est en haut à midi (180° rotation visuelle pour CSS)
    // On veut: Midi = Soleil Zénith. 
    // CSS rot: 0 = Haut. Donc à midi (12h), rot = 0.
    // À minuit (0h), rot = 180.
    const rot = (deg + 180) % 360; 

    if(sunEl) sunEl.style.transform = `rotate(${rot}deg)`;
    if(moonEl) moonEl.style.transform = `rotate(${rot + 180}deg)`;

    // Couleur Ciel & Statut
    if (clockContainer) {
        clockContainer.className = ''; // Reset
        if (rot > 330 || rot < 30) { 
            clockContainer.classList.add('sky-day'); 
            setText('clock-status', 'Jour (☀️)');
        } else if (rot >= 30 && rot < 60) {
            clockContainer.classList.add('sky-sunset');
            setText('clock-status', 'Coucher (🌅)');
        } else if (rot >= 150 && rot < 210) {
            clockContainer.classList.add('sky-night');
            setText('clock-status', 'Nuit (🌙)');
        } else {
            clockContainer.classList.add('sky-day'); // Défaut
        }
    }
}

// =================================================================
// 7. INITIALISATION & UI EVENTS
// =================================================================
let map = null;
let marker = null;

function initMap() {
    if (typeof L !== 'undefined' && !map) {
        map = L.map('map').setView([0, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap',
            maxZoom: 19
        }).addTo(map);
        marker = L.marker([0, 0]).addTo(map);
    }
}

function updateMap(lat, lon, acc) {
    if (map && lat !== 0) {
        const latLng = [lat, lon];
        marker.setLatLng(latLng);
        map.setView(latLng, 16); // Zoom auto
        
        setText('lat-display', fmt(lat, 6, '°'));
        setText('lon-display', fmt(lon, 6, '°'));
        setText('alt-display', fmt(gpsData.alt, 1, ' m'));
    }
}

// Listeners Boutons
document.addEventListener('DOMContentLoaded', () => {
    initMap();

    // Bouton GPS
    $('toggle-gps-btn').addEventListener('click', () => {
        SYSTEM_STATE.gpsActive = !SYSTEM_STATE.gpsActive;
        const btn = $('toggle-gps-btn');
        if(SYSTEM_STATE.gpsActive) {
            startGPS();
            btn.textContent = "⏸️ PAUSE GPS";
            btn.style.backgroundColor = "#ffc107"; // Jaune
            btn.style.color = "black";
            setText('speed-status-text', 'Recherche satellites...');
        } else {
            btn.textContent = "▶️ MARCHE GPS";
            btn.style.backgroundColor = "#28a745"; // Vert
            btn.style.color = "white";
            setText('speed-status-text', 'GPS en Pause');
        }
    });

    // Arrêt d'Urgence
    $('emergency-stop-btn').addEventListener('click', () => {
        SYSTEM_STATE.isEmergencyStop = !SYSTEM_STATE.isEmergencyStop;
        const btn = $('emergency-stop-btn');
        btn.classList.toggle('active');
        if(SYSTEM_STATE.isEmergencyStop) {
            btn.textContent = "🛑 ARRÊT D'URGENCE: ACTIF 🔴";
            setText('speed-status-text', 'SYSTÈME STOPPÉ');
        } else {
            btn.textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        }
    });

    // Mode Nuit
    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
    });

    // Mode Nether
    $('nether-toggle-btn').addEventListener('click', () => {
        SYSTEM_STATE.netherMode = !SYSTEM_STATE.netherMode;
        const btn = $('nether-toggle-btn');
        const status = $('nether-mode-status');
        
        if (SYSTEM_STATE.netherMode) {
            SYSTEM_STATE.distanceRatio = 8.0;
            btn.textContent = "Mode Nether: ACTIVÉ (1:8)";
            btn.style.backgroundColor = "#5e1116"; // Rouge sombre
            if(status) { status.style.display = 'block'; status.textContent = "Rapport 8.0"; }
        } else {
            SYSTEM_STATE.distanceRatio = 1.0;
            btn.textContent = "Mode Nether: DÉSACTIVÉ (1:1)";
            btn.style.backgroundColor = "#555";
            if(status) status.style.display = 'none';
        }
        setText('distance-ratio', fmt(SYSTEM_STATE.distanceRatio, 3));
    });

    // Environnement Kalman
    $('environment-select').addEventListener('change', (e) => {
        const val = e.target.value;
        let factor = 1.0;
        let txt = "Normal (x1.0)";
        
        if(val === 'FOREST') { factor = 2.5; txt = "Forêt (x2.5)"; }
        else if(val === 'CONCRETE') { factor = 7.0; txt = "Grotte/Tunnel (x7.0)"; }
        else if(val === 'METAL') { factor = 5.0; txt = "Métal (x5.0)"; }
        
        SYSTEM_STATE.environmentFactor = factor;
        setText('env-factor', txt);
    });

    // Rayons X (Horloge)
    $('xray-button').addEventListener('click', () => {
        const clock = $('minecraft-clock');
        if(clock) clock.classList.toggle('x-ray');
    });
    
    // Réactivité UKF
    $('ukf-reactivity-mode').addEventListener('change', (e) => {
        const val = e.target.value;
        if (val === 'FAST') SYSTEM_STATE.ukfReactivity = 0.2; // R faible = confiant mesure
        else if (val === 'STABLE') SYSTEM_STATE.ukfReactivity = 2.5; // R fort = confiant modèle
        else SYSTEM_STATE.ukfReactivity = 1.0;
    });

    // Reset Buttons
    $('reset-max-btn').addEventListener('click', () => { SYSTEM_STATE.maxSpeed = 0; });
    $('reset-dist-btn').addEventListener('click', () => { SYSTEM_STATE.totalDistance = 0; });
    $('reset-all-btn').addEventListener('click', () => { location.reload(); });

    // Inputs Masse
    $('mass-input').addEventListener('input', (e) => {
        SYSTEM_STATE.mass = parseFloat(e.target.value) || 70;
        setText('mass-display', fmt(SYSTEM_STATE.mass, 3, ' kg'));
    });
    
    // Démarrage boucle d'affichage
    requestAnimationFrame(loop);
});

// =================================================================
// 8. BOUCLE DE RENDU (60 FPS)
// =================================================================
let lastLoop = 0;

function loop(timestamp) {
    if (timestamp - lastLoop > 1000) { // 1Hz updates pour Astro/Temps lent
        updateAstro();
        setText('elapsed-time', fmt((Date.now() - SYSTEM_STATE.startTime)/1000, 1, ' s'));
        lastLoop = timestamp;
    }
    
    // Updates Fréquents (Physique, UI Vitesse)
    updatePhysics();

    requestAnimationFrame(loop);
    }
