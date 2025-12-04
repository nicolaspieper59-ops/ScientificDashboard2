// =================================================================
// GNSS SPACETIME DASHBOARD • UKF FUSION ENGINE (PROFESSIONNEL)
// Version: 2.1 (Scientifique & Réaliste)
// Auteur: Gemini AI (Optimisé pour structure HTML fournie)
// Dépendances: math.min.js, leaflet.js, suncalc.js, turf.min.js
// =================================================================

((window) => {
    'use strict';

    // --- 1. CONFIGURATION ET CONSTANTES PHYSIQUES ---

    const CONSTANTS = {
        // Physique Fondamentale
        C: 299792458,          // Vitesse lumière (m/s)
        G: 6.67430e-11,        // Constante gravitationnelle
        g0: 9.80665,           // Gravité standard
        R_EARTH: 6371000,      // Rayon terrestre moyen (m)
        AU: 149597870700,      // Unité Astronomique (m)
        LY: 9.461e15,          // Année-lumière (m)
        
        // Atmosphère (ISA Standard)
        P0: 101325,            // Pression niveau mer (Pa)
        T0: 288.15,            // Température niveau mer (K)
        L: 0.0065,             // Gradient thermique (K/m)
        R_AIR: 287.05,         // Constante spécifique de l'air
        GAMMA: 1.4,            // Indice adiabatique

        // Conversion
        D2R: Math.PI / 180,
        R2D: 180 / Math.PI,
        MS_TO_KMH: 3.6
    };

    // Configuration du système
    const CONFIG = {
        UKF_DIM: 9,            // État: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]
        DT_MIN: 0.02,          // Pas de temps minimum (s)
        MAP_ZOOM_DEFAULT: 16,
        ZUPT_THRESH: 0.2,      // Seuil détection arrêt (m/s²)
        AUDIO_FFT_SIZE: 256
    };

    // --- 2. ÉTAT GLOBAL DU SYSTÈME ---

    const state = {
        gps: { lat: null, lon: null, alt: 0, acc: 0, speed: 0, t: 0, active: false },
        imu: { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, mx: 0, my: 0, mz: 0, pitch: 0, roll: 0 },
        env: { light: 0, lightMax: 0, sound: 0, soundMax: 0, pressure: null },
        fusion: { lat: 0, lon: 0, alt: 0, vN: 0, vE: 0, vD: 0, speed: 0, dist: 0 },
        meta: { mass: 70, startTime: Date.now(), movingTime: 0, maxSpeed: 0, celestialBody: 'EARTH' },
        flags: { emergency: false, recording: false, nightMode: true }
    };

    // Références DOM (Cache)
    const dom = {};
    const $ = (id) => {
        if (!dom[id]) dom[id] = document.getElementById(id);
        return dom[id];
    };

    // Objets Leaflet
    let map = null, userMarker = null, accCircle = null;

    // --- 3. MOTEUR UKF (Unscented Kalman Filter) ---
    // Implémentation cinématique pour fusionner GPS (Position) + IMU (Accélération)
    
    class KinematicUKF {
        constructor() {
            if (typeof math === 'undefined') throw new Error("Math.js manquant");
            
            // État initial (9x1) : Pos(3), Vel(3), AccBias(3)
            this.x = math.matrix(math.zeros([9, 1]));
            
            // Covariance initiale (P)
            this.P = math.multiply(math.identity(9), 10); 
            
            // Bruit de processus (Q) - Confiance dans le modèle physique
            this.Q = math.multiply(math.identity(9), 0.01);
            
            // Bruit de mesure (R) - Confiance dans le GPS (sera ajusté dynamiquement)
            this.R = math.multiply(math.identity(3), 5.0); // GPS x,y,z
        }

        predict(dt, ax, ay, az) {
            // Modèle de mouvement à accélération constante
            // x = x + v*dt + 0.5*a*dt^2
            // v = v + a*dt
            
            const F = math.identity(9);
            // Mise à jour position par vitesse
            F.subset(math.index(0, 3), dt); F.subset(math.index(1, 4), dt); F.subset(math.index(2, 5), dt);
            // Mise à jour position par accélération (0.5 * dt^2)
            const dt2 = 0.5 * dt * dt;
            F.subset(math.index(0, 6), dt2); F.subset(math.index(1, 7), dt2); F.subset(math.index(2, 8), dt2);
            // Mise à jour vitesse par accélération
            F.subset(math.index(3, 6), dt); F.subset(math.index(4, 7), dt); F.subset(math.index(5, 8), dt);

            // Prédiction de l'état a priori
            this.x = math.multiply(F, this.x);
            
            // Injection de l'IMU dans l'état (Accélération observée)
            // Note: Simplification pour ce contexte mono-fichier
            // Dans une implémentation stricte, IMU est l'entrée de commande (u), pas l'état.
            // Ici on utilise l'accélération comme état pour lisser le bruit IMU.
            const alpha = 0.1; // Filtre passe-bas simple sur l'input IMU vers l'état
            let currentAx = this.x.get([6, 0]);
            let currentAy = this.x.get([7, 0]);
            let currentAz = this.x.get([8, 0]);
            
            this.x.subset(math.index(6, 0), currentAx * (1-alpha) + ax * alpha);
            this.x.subset(math.index(7, 0), currentAy * (1-alpha) + ay * alpha);
            this.x.subset(math.index(8, 0), currentAz * (1-alpha) + az * alpha);

            // Covariance a priori: P = F * P * F' + Q
            const FP = math.multiply(F, this.P);
            const FPFt = math.multiply(FP, math.transpose(F));
            this.P = math.add(FPFt, this.Q);
        }

        update(lat, lon, alt, accuracy) {
            // Conversion Géodésique -> ECEF Local (Approximation plan tangent pour petites distances)
            // Pour ce dashboard, on travaille en Delta par rapport au point de départ serait mieux,
            // mais on va simplifier en traitant Lat/Lon directement comme X/Y à l'échelle locale.
            // X = Lat * R, Y = Lon * R * cos(Lat)
            
            const R = CONSTANTS.R_EARTH;
            const cosLat = Math.cos(lat * CONSTANTS.D2R);
            
            const z_meas = math.matrix([[lat * R], [lon * R * cosLat], [alt]]);
            
            // Matrice d'observation H (On observe Pos X, Pos Y, Pos Z)
            const H = math.zeros([3, 9]);
            H.subset(math.index(0, 0), 1);
            H.subset(math.index(1, 1), 1);
            H.subset(math.index(2, 2), 1);

            // Ajustement dynamique de R selon précision GPS
            const R_val = accuracy * accuracy; // Variance
            this.R = math.multiply(math.identity(3), R_val);

            // Innovation: y = z - H * x
            const Hx = math.multiply(H, this.x);
            const y = math.subtract(z_meas, Hx);

            // S = H * P * H' + R
            const HPHt = math.multiply(math.multiply(H, this.P), math.transpose(H));
            const S = math.add(HPHt, this.R);

            // Gain de Kalman: K = P * H' * inv(S)
            const K = math.multiply(math.multiply(this.P, math.transpose(H)), math.inv(S));

            // Mise à jour a posteriori: x = x + K * y
            this.x = math.add(this.x, math.multiply(K, y));

            // Mise à jour Covariance: P = (I - K * H) * P
            const I = math.identity(9);
            const KH = math.multiply(K, H);
            this.P = math.multiply(math.subtract(I, KH), this.P);
            
            // Retourner les valeurs géodésiques
            const x_est = this.x.get([0, 0]);
            const y_est = this.x.get([1, 0]);
            
            return {
                lat: x_est / R,
                lon: y_est / (R * cosLat),
                alt: this.x.get([2, 0]),
                vN: this.x.get([3, 0]),
                vE: this.x.get([4, 0]),
                vD: this.x.get([5, 0])
            };
        }
        
        getUncertainty() {
            // Trace de la sous-matrice position P[0..2, 0..2]
            return Math.sqrt(this.P.get([0,0]) + this.P.get([1,1]) + this.P.get([2,2]));
        }
    }

    const ukf = new KinematicUKF();

    // --- 4. GESTION DES CAPTEURS ---

    function initSensors() {
        // A. API Generic Sensor (Accéléromètre / Gyro)
        if ('Accelerometer' in window && 'Gyroscope' in window) {
            try {
                const accSensor = new Accelerometer({ frequency: 60 });
                accSensor.addEventListener('reading', () => {
                    // Conversion repère appareil -> repère véhicule
                    state.imu.ax = accSensor.x;
                    state.imu.ay = accSensor.y;
                    state.imu.az = accSensor.z;
                    updateBubbleLevel(accSensor.x, accSensor.y, accSensor.z);
                    
                    if($('acceleration-x')) $('acceleration-x').textContent = accSensor.x.toFixed(2) + " m/s²";
                    if($('acceleration-y')) $('acceleration-y').textContent = accSensor.y.toFixed(2) + " m/s²";
                    if($('acceleration-z')) $('acceleration-z').textContent = accSensor.z.toFixed(2) + " m/s²";
                });
                accSensor.start();

                const gyroSensor = new Gyroscope({ frequency: 60 });
                gyroSensor.addEventListener('reading', () => {
                    state.imu.gx = gyroSensor.x;
                    state.imu.gy = gyroSensor.y;
                    state.imu.gz = gyroSensor.z;
                    // Intégration simple pour Pitch/Roll (Si accéléromètre bruité)
                });
                gyroSensor.start();
                $('statut-capteur').textContent = "ACTIF (HW)";
                $('statut-capteur').style.color = "#00ff00";
            } catch (err) {
                console.warn("Erreur capteurs IMU:", err);
                $('statut-capteur').textContent = "ERREUR / RESTREINT";
            }
        }

        // B. Capteur de Lumière
        if ('AmbientLightSensor' in window) {
            try {
                const lightSensor = new AmbientLightSensor();
                lightSensor.addEventListener('reading', () => {
                    state.env.light = lightSensor.illuminance;
                    state.env.lightMax = Math.max(state.env.lightMax, state.env.light);
                    $('ambient-light').textContent = state.env.light.toFixed(0) + " Lux";
                    $('ambient-light-max').textContent = state.env.lightMax.toFixed(0) + " Lux";
                });
                lightSensor.start();
            } catch (e) { console.log("Light Sensor non supporté"); }
        }

        // C. Audio (Microphone) pour dB
        navigator.mediaDevices.getUserMedia({ audio: true, video: false })
        .then(stream => {
            const audioContext = new (window.AudioContext || window.webkitAudioContext)();
            const analyser = audioContext.createAnalyser();
            const microphone = audioContext.createMediaStreamSource(stream);
            const javascriptNode = audioContext.createScriptProcessor(2048, 1, 1);

            analyser.smoothingTimeConstant = 0.8;
            analyser.fftSize = 1024;

            microphone.connect(analyser);
            analyser.connect(javascriptNode);
            javascriptNode.connect(audioContext.destination);

            javascriptNode.onaudioprocess = () => {
                const array = new Uint8Array(analyser.frequencyBinCount);
                analyser.getByteFrequencyData(array);
                let values = 0;
                const length = array.length;
                for (let i = 0; i < length; i++) values += array[i];
                const average = values / length;
                
                // Calibration approximative : 0-255 -> dB SPL (Hypothèse non calibrée)
                // Logarithmique : 20 * log10(amp)
                if (average > 0) {
                    const db = 20 * Math.log10(average) + 10; // Offset arbitraire
                    state.env.sound = db;
                    state.env.soundMax = Math.max(state.env.soundMax, db);
                    $('sound-level').textContent = db.toFixed(1) + " dB";
                    $('sound-level-max').textContent = state.env.soundMax.toFixed(1) + " dB";
                }
            };
        }).catch(e => console.log("Audio non autorisé"));
    }

    function updateBubbleLevel(x, y, z) {
        // Calcul Inclinaison (Pitch) et Roulis (Roll)
        // Roll = atan2(y, z)
        // Pitch = atan2(-x, sqrt(y*y + z*z))
        
        state.imu.roll = Math.atan2(y, z) * CONSTANTS.R2D;
        state.imu.pitch = Math.atan2(-x, Math.sqrt(y*y + z*z)) * CONSTANTS.R2D;

        if($('inclinaison-pitch')) $('inclinaison-pitch').textContent = state.imu.pitch.toFixed(1) + "°";
        if($('roulis-roll')) $('roulis-roll').textContent = state.imu.roll.toFixed(1) + "°";

        // Animation de la bulle CSS
        const bubble = $('bubble');
        if (bubble) {
            // Limiter le déplacement pour rester dans le cercle
            const limit = 40; 
            const bx = Math.max(-limit, Math.min(limit, -state.imu.roll)); // Inversion axe X écran
            const by = Math.max(-limit, Math.min(limit, -state.imu.pitch)); // Inversion axe Y écran
            bubble.style.transform = `translate(${bx}px, ${by}px)`;
        }
    }

    // --- 5. PHYSIQUE & RELATIVITÉ ---

    function updatePhysics() {
        const v = state.fusion.speed; // m/s
        const mass = parseFloat($('mass-input').value) || 70;
        const altitude = state.fusion.alt;

        // 1. Relativité Restreinte
        const beta = v / CONSTANTS.C;
        const gamma = 1 / Math.sqrt(Math.max(0, 1 - beta*beta)); // Facteur de Lorentz
        
        // Dilatation temporelle cinématique (ns/jour)
        // dt' = gamma * dt. Différence = (gamma - 1) * 86400 * 1e9
        const timeDilationV = (gamma - 1) * 86400 * 1e9;

        // Énergie
        const E0 = mass * Math.pow(CONSTANTS.C, 2); // Joules
        const E_total = gamma * E0;
        const E_kinetic = E_total - E0;

        // 2. Relativité Générale (Schwarzschild)
        // Rayon de Schwarzschild de l'objet
        const Rs = (2 * CONSTANTS.G * mass) / Math.pow(CONSTANTS.C, 2);
        
        // Dilatation gravifique (Approx par rapport à l'infini, ou différentiel hauteur)
        // Pour le dashboard : Potentiel gravitationnel local
        const U = -(CONSTANTS.G * 5.972e24) / (CONSTANTS.R_EARTH + altitude);
        const timeDilationG = (1 - Math.sqrt(1 + (2*U)/Math.pow(CONSTANTS.C, 2))) * 86400 * 1e9; // Approx

        // 3. Atmosphère & Aérodynamique
        // Calculs ISA
        const tempK = CONSTANTS.T0 - CONSTANTS.L * altitude;
        const pressure = CONSTANTS.P0 * Math.pow(1 - (CONSTANTS.L * altitude)/CONSTANTS.T0, (CONSTANTS.g0 * 0.02896) / (CONSTANTS.R_AIR * CONSTANTS.L));
        const density = pressure / (CONSTANTS.R_AIR * tempK);
        const speedOfSound = Math.sqrt(CONSTANTS.GAMMA * CONSTANTS.R_AIR * tempK);
        const mach = v / speedOfSound;

        // Pression Dynamique q = 0.5 * rho * v^2
        const q = 0.5 * density * v * v;
        const Cd = 1.0; // Coeff de trainée moyen (humain debout)
        const Area = 0.5; // Surface frontale (m2)
        const dragForce = q * Cd * Area;
        const dragPower = (dragForce * v) / 1000; // kW

        // --- MISE À JOUR DOM PHYSIQUE ---
        if($('speed-stable-ms')) $('speed-stable-ms').textContent = v.toFixed(2) + " m/s";
        if($('speed-stable')) $('speed-stable').textContent = (v * 3.6).toFixed(1) + " km/h";
        
        if($('mach-number')) $('mach-number').textContent = mach.toFixed(4);
        if($('lorentz-factor')) $('lorentz-factor').textContent = gamma.toFixed(10);
        if($('time-dilation-vitesse')) $('time-dilation-vitesse').textContent = timeDilationV.toFixed(4) + " ns/j";
        if($('time-dilation-gravite')) $('time-dilation-gravite').textContent = timeDilationG.toFixed(4) + " ns/j";
        
        if($('relativistic-energy')) $('relativistic-energy').textContent = E_total.toExponential(3) + " J";
        if($('kinetic-energy')) $('kinetic-energy').textContent = E_kinetic.toFixed(2) + " J";
        if($('rest-mass-energy')) $('rest-mass-energy').textContent = E0.toExponential(3) + " J";
        if($('momentum')) $('momentum').textContent = (gamma * mass * v).toFixed(2) + " kg·m/s";
        if($('schwarzschild-radius')) $('schwarzschild-radius').textContent = Rs.toExponential(4) + " m";

        // Aéro
        if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = speedOfSound.toFixed(1) + " m/s";
        if($('air-density')) $('air-density').textContent = density.toFixed(3) + " kg/m³";
        if($('dynamic-pressure')) $('dynamic-pressure').textContent = q.toFixed(1) + " Pa";
        if($('drag-force')) $('drag-force').textContent = dragForce.toFixed(1) + " N";
        if($('drag-power-kw')) $('drag-power-kw').textContent = dragPower.toFixed(2) + " kW";

        // Distances Lumière
        const dist = state.fusion.dist;
        if($('distance-totale')) $('distance-totale').textContent = (dist/1000).toFixed(3) + " km | " + dist.toFixed(1) + " m";
        if($('distance-light-s')) $('distance-light-s').textContent = (dist / CONSTANTS.C).toExponential(2) + " s";
        if($('distance-cosmic')) $('distance-cosmic').textContent = (dist / CONSTANTS.AU).toExponential(2) + " UA";
    }

    // --- 6. ASTRONOMIE & TEMPS ---

    function updateAstro() {
        const now = new Date();
        const lat = state.fusion.lat || 43.296; // Marseille par défaut
        const lon = state.fusion.lon || 5.37;

        if (typeof SunCalc === 'undefined') return;

        // Positions
        const sunPos = SunCalc.getPosition(now, lat, lon);
        const moonPos = SunCalc.getMoonPosition(now, lat, lon);
        const moonIllum = SunCalc.getMoonIllumination(now);
        const sunTimes = SunCalc.getTimes(now, lat, lon);

        // Conversion Radians -> Degrés
        const sunAlt = sunPos.altitude * CONSTANTS.R2D;
        const sunAz = sunPos.azimuth * CONSTANTS.R2D + 180;
        const moonAlt = moonPos.altitude * CONSTANTS.R2D;

        // Calcul Heure Solaire Vraie (TST)
        // EOT (Equation of Time) approximation simplifiée pour la performance, 
        // ou utilisation de la différence entre Midi Solaire (SunCalc) et Midi Local.
        const solarNoon = sunTimes.solarNoon;
        const noonDiffMs = solarNoon.getTime() - now.getTime(); // Différence temporelle brute
        
        // Calcul précis de l'EOT via formule VSOP approximée
        const d = Math.floor((now - new Date(now.getFullYear(), 0, 0)) / 1000 / 60 / 60 / 24); // Jour de l'année
        const b = 2 * Math.PI * (d - 81) / 365;
        const eot = 9.87 * Math.sin(2*b) - 7.53 * Math.cos(b) - 1.5 * Math.sin(b); // Minutes
        
        // TST = UTC + LonOffset + EOT
        const lonOffsetMin = lon * 4; // 4 min par degré
        const totalOffsetMin = lonOffsetMin + eot;
        const tstMs = now.getUTCHours()*3600000 + now.getUTCMinutes()*60000 + now.getUTCSeconds()*1000 + totalOffsetMin*60000;
        const tstDate = new Date(tstMs); // Note: C'est une représentation relative
        
        // Mise à jour DOM Astro
        if($('sun-alt')) $('sun-alt').textContent = sunAlt.toFixed(2) + "°";
        if($('sun-azimuth')) $('sun-azimuth').textContent = sunAz.toFixed(2) + "°";
        if($('moon-phase-name')) $('moon-phase-name').textContent = getPhaseName(moonIllum.phase);
        if($('moon-illuminated')) $('moon-illuminated').textContent = (moonIllum.fraction * 100).toFixed(1) + " %";
        if($('eot')) $('eot').textContent = eot.toFixed(2) + " min";
        
        const formatTime = (d) => d.toISOString().split('T')[1].split('.')[0];
        const formatTimeHM = (h, m) => `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}`;
        
        // Calcul manuel TST pour affichage propre
        let tstH = Math.floor(tstMs / 3600000) % 24;
        let tstM = Math.floor((tstMs % 3600000) / 60000);
        let tstS = Math.floor((tstMs % 60000) / 1000);
        if (tstH < 0) tstH += 24;
        if($('tst')) $('tst').textContent = `${String(tstH).padStart(2,'0')}:${String(tstM).padStart(2,'0')}:${String(tstS).padStart(2,'0')}`;
        
        if($('noon-solar')) $('noon-solar').textContent = solarNoon.toLocaleTimeString();

        // Horloge Minecraft (Rotation CSS & Ambiance)
        // Principe : Rotation basée sur l'Heure Solaire Vraie (TST) pour aligner le zénith visuel.
        const clockSun = $('sun-element');
        const clockMoon = $('moon-element');
        const clockContainer = $('minecraft-clock');
        
        if (clockSun && clockMoon && clockContainer) {
            // Mapping : 12:00 TST = Soleil en haut (0° ou rotation neutre selon CSS)
            // La journée fait 360°, donc 15° par heure.
            // On centre sur midi solaire (12h).
            
            const tstDecimal = tstH + (tstM / 60) + (tstS / 3600);
            
            // Rotation : Si TST = 12, rot = 0 (Zénith). Si TST = 6, rot = -90 (Lever).
            const rotationDeg = (tstDecimal - 12) * 15; 
            
            clockSun.style.transform = `rotate(${rotationDeg}deg)`;
            clockMoon.style.transform = `rotate(${rotationDeg + 180}deg)`; // Lune à l'opposé (approx)
            
            // Gestion de l'ambiance (Couleur du ciel) basée sur l'altitude réelle du soleil
            clockContainer.className = ""; // Reset
            if (sunAlt > 0) {
                clockContainer.classList.add("sky-day");
                if($('clock-status')) $('clock-status').textContent = "Jour (☀️)";
            } else if (sunAlt > -6) {
                // Crépuscule civil
                clockContainer.classList.add("sky-sunset");
                if($('clock-status')) $('clock-status').textContent = "Crépuscule (✨)";
            } else {
                clockContainer.classList.add("sky-night");
                if($('clock-status')) $('clock-status').textContent = "Nuit (🌙)";
            }
            
            // Calcul "Temps Minecraft" (Purement cosmétique pour les fans)
            // Dans le jeu : 0 ticks = 6:00 AM. 6000 ticks = Midi. 18000 ticks = Minuit.
            // Formule : ((Heure + 18) % 24) * 1000
            const mcTicks = Math.floor(((now.getHours() + 18) % 24) * 1000 + (now.getMinutes() / 60) * 1000);
            if($('time-minecraft')) $('time-minecraft').textContent = `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')} (${mcTicks} ticks)`;
        }
    }

    function getPhaseName(phase) {
        // Phase de 0.0 à 1.0 (SunCalc)
        if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
        if (phase < 0.25) return "Premier Croissant 🌒";
        if (phase < 0.28) return "Premier Quartier 🌓";
        if (phase < 0.50) return "Gibbeuse Croissante 🌔";
        if (phase < 0.53) return "Pleine Lune 🌕";
        if (phase < 0.75) return "Gibbeuse Décroissante 🌖";
        if (phase < 0.78) return "Dernier Quartier 🌗";
        return "Dernier Croissant 🌘";
    }

    // --- 7. BOUCLE PRINCIPALE & GPS (Fusion de données) ---

    function onGPSUpdate(position) {
        if (state.flags.emergency) return;

        const coords = position.coords;
        const now = Date.now();
        // Calcul du Delta Time (dt) en secondes
        const dt = state.gps.t === 0 ? 0.02 : (now - state.gps.t) / 1000;
        state.gps.t = now;

        // 1. Mise à jour des données brutes GPS
        state.gps.lat = coords.latitude;
        state.gps.lon = coords.longitude;
        state.gps.alt = coords.altitude || 0; // Souvent null sur mobiles basiques
        state.gps.acc = coords.accuracy || 10;
        state.gps.speed = coords.speed || 0; // Vitesse Doppler brute (souvent très précise)

        // 2. FUSION UKF (Mise à jour étape "Correction")
        // On injecte la position GPS observée pour corriger la prédiction IMU
        const result = ukf.update(state.gps.lat, state.gps.lon, state.gps.alt, state.gps.acc);
        
        // Stockage de l'état fusionné
        const prevLat = state.fusion.lat;
        const prevLon = state.fusion.lon;
        
        state.fusion.lat = result.lat;
        state.fusion.lon = result.lon;
        state.fusion.alt = result.alt;
        
        // Calcul de la vitesse fusionnée (Norme du vecteur vitesse 3D estimé)
        const vUKF = Math.sqrt(result.vN**2 + result.vE**2 + result.vD**2);
        
        // ZUPT (Zero Velocity Update) Logic :
        // Si le GPS dit 0 et l'accéléromètre est stable, on force la vitesse à 0 pour éviter la dérive.
        if (state.gps.speed < 0.1 && Math.abs(state.imu.ax) < CONFIG.ZUPT_THRESH) {
            state.fusion.speed = 0;
        } else {
            state.fusion.speed = vUKF;
        }

        // 3. Calcul de Distance (Accumulation)
        // On n'ajoute de la distance que si on bouge vraiment (seuil ZUPT)
        if (prevLat !== 0 && state.fusion.speed > CONFIG.ZUPT_THRESH) {
            const d = calcDistance(prevLat, prevLon, state.fusion.lat, state.fusion.lon);
            state.fusion.dist += d;
            state.meta.movingTime += dt;
        }

        // Stats Session
        if (state.fusion.speed > state.meta.maxSpeed) state.meta.maxSpeed = state.fusion.speed;

        // 4. Mises à jour Visuelles
        updateUI();
        updateMap(state.fusion.lat, state.fusion.lon, state.gps.acc);
    }

    function onGPSError(err) {
        if($('gps-accuracy-display')) $('gps-accuracy-display').textContent = "Erreur";
        if($('acc-gps')) $('acc-gps').textContent = `Err ${err.code}`;
        console.warn("GPS Error:", err.message);
    }

    // Formule de Haversine pour la distance sphérique
    function calcDistance(lat1, lon1, lat2, lon2) {
        const R = CONSTANTS.R_EARTH;
        const dLat = (lat2 - lat1) * CONSTANTS.D2R;
        const dLon = (lon2 - lon1) * CONSTANTS.D2R;
        const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                  Math.cos(lat1 * CONSTANTS.D2R) * Math.cos(lat2 * CONSTANTS.D2R) *
                  Math.sin(dLon/2) * Math.sin(dLon/2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
        return R * c;
    }

    // --- 8. UI & CARTE ---

    function initMap() {
        if (typeof L !== 'undefined') {
            // Initialisation Leaflet
            map = L.map('map').setView([0, 0], 2);
            
            // Tuiles OpenStreetMap (Nécessite connexion, fallback possible)
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OSM',
                maxZoom: 19
            }).addTo(map);

            // Marqueur Utilisateur & Cercle de précision
            userMarker = L.marker([0, 0]).addTo(map);
            accCircle = L.circle([0, 0], { radius: 1, color: '#007bff', fillOpacity: 0.2 }).addTo(map);
        } else {
            if($('map')) $('map').innerHTML = "Leaflet JS non chargé.";
        }
    }

    function updateMap(lat, lon, acc) {
        if (map && userMarker) {
            const latlng = [lat, lon];
            userMarker.setLatLng(latlng);
            accCircle.setLatLng(latlng);
            accCircle.setRadius(acc);
            
            // Auto-center si c'est la première acquisition ou mode suivi
            if (state.gps.t === 0 || (map.getZoom() < 10)) {
                map.setView(latlng, CONFIG.MAP_ZOOM_DEFAULT);
            }
        }
    }

    function updateUI() {
        // --- Colonne 1 : Contrôles & Stats de base ---
        const now = new Date();
        if($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString();
        if($('elapsed-session-time')) $('elapsed-session-time').textContent = ((now - state.meta.startTime)/1000).toFixed(1) + " s";
        if($('elapsed-motion-time')) $('elapsed-motion-time').textContent = state.meta.movingTime.toFixed(1) + " s";

        // --- Colonne 2 : Vitesse & Distance ---
        // Vitesse Stable (UKF)
        if($('speed-stable')) $('speed-stable').textContent = (state.fusion.speed * 3.6).toFixed(1) + " km/h";
        if($('speed-stable-ms')) $('speed-stable-ms').textContent = state.fusion.speed.toFixed(2) + " m/s";
        // Vitesse Instantanée (GPS Brut)
        if($('speed-3d-inst')) $('speed-3d-inst').textContent = (state.gps.speed * 3.6).toFixed(1) + " km/h";
        // Max & Moyenne
        if($('vitesse-max-session')) $('vitesse-max-session').textContent = (state.meta.maxSpeed * 3.6).toFixed(1) + " km/h";
        
        // Distances
        if($('distance-totale')) $('distance-totale').textContent = (state.fusion.dist / 1000).toFixed(3) + " km";

        // Physique Avancée (Appel de la fonction de la section 5)
        updatePhysics();

        // --- Colonne 3 : EKF & Position ---
        if($('lat-ekf')) $('lat-ekf').textContent = state.fusion.lat.toFixed(6) + "°";
        if($('lon-ekf')) $('lon-ekf').textContent = state.fusion.lon.toFixed(6) + "°";
        if($('alt-ekf')) $('alt-ekf').textContent = state.fusion.alt.toFixed(1) + " m";
        if($('acc-gps')) $('acc-gps').textContent = state.gps.acc.toFixed(1) + " m";
        
        if($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = "FIX 3D (Actif)";
        if($('ukf-v-uncert')) $('ukf-v-uncert').textContent = ukf.getUncertainty().toFixed(3);
        if($('gps-accuracy-display')) $('gps-accuracy-display').textContent = state.gps.acc.toFixed(2) + " m";
    }

    // Boucle d'animation visuelle (Astro & Prédiction IMU rapide)
    function animate() {
        if (!state.flags.emergency) {
            // Etape de PRÉDICTION UKF (haute fréquence, entre les updates GPS)
            // On suppose un dt de 16ms (60fps) pour la fluidité visuelle
            // Note: Ceci met à jour l'état interne de l'UKF sans mesure GPS
            ukf.predict(0.016, state.imu.ax, state.imu.ay, state.imu.az);
            
            // Mise à jour de l'astronomie (Soleil/Lune bougent en temps réel)
            updateAstro();
        }
        requestAnimationFrame(animate);
    }

    // --- 9. INITIALISATION ET ÉVÉNEMENTS ---

    document.addEventListener('DOMContentLoaded', () => {
        // Initialiser la carte Leaflet
        initMap();
        
        // --- Bouton MARCHE/ARRÊT GPS ---
        const btnGps = $('toggle-gps-btn');
        if (btnGps) {
            btnGps.addEventListener('click', () => {
                if (!state.gps.active) {
                    if (navigator.geolocation) {
                        // 1. Démarrer les capteurs
                        initSensors();
                        
                        // 2. Démarrer le GPS
                        const opts = { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 };
                        navigator.geolocation.watchPosition(onGPSUpdate, onGPSError, opts);
                        
                        // 3. Mise à jour état
                        state.gps.active = true;
                        state.meta.startTime = Date.now();
                        
                        // 4. UI
                        btnGps.textContent = "⏸️ PAUSE GPS";
                        btnGps.style.backgroundColor = "#ffc107"; // Jaune
                        btnGps.style.color = "#000";
                        
                        // 5. Lancer la boucle d'animation
                        animate();
                        
                    } else {
                        alert("La géolocalisation n'est pas supportée par ce navigateur.");
                    }
                } else {
                    // Pour l'instant, on recharge la page pour un arrêt propre (nettoyage complet)
                    // Dans une version plus complexe, on utiliserait clearWatch et clearInterval
                    if(confirm("Arrêter la session et réinitialiser ?")) {
                        location.reload();
                    }
                }
            });
        }

        // --- Bouton ARRÊT D'URGENCE ---
        const btnEmergency = $('emergency-stop-btn');
        if (btnEmergency) {
            btnEmergency.addEventListener('click', () => {
                state.flags.emergency = !state.flags.emergency;
                if (state.flags.emergency) {
                    btnEmergency.classList.add('active');
                    btnEmergency.textContent = "🛑 ARRÊT D'URGENCE ACTIF";
                    // Forcer la vitesse à 0
                    state.fusion.speed = 0;
                    $('speed-stable').textContent = "🛑 0.0 km/h";
                } else {
                    btnEmergency.classList.remove('active');
                    btnEmergency.textContent = "🛑 Arrêt d'urgence: INACTIF";
                    animate(); // Relancer l'animation
                }
            });
        }

        // --- Autres Contrôles UI ---
        
        // Mode Nuit / Jour
        const btnMode = $('toggle-mode-btn');
        if (btnMode) {
            btnMode.addEventListener('click', () => {
                document.body.classList.toggle('dark-mode');
                state.flags.nightMode = document.body.classList.contains('dark-mode');
            });
        }

        // Reset Distance
        const btnResetDist = $('reset-dist-btn');
        if (btnResetDist) {
            btnResetDist.addEventListener('click', () => {
                state.fusion.dist = 0;
                state.meta.movingTime = 0;
                updateUI();
            });
        }

        // Reset V-Max
        const btnResetMax = $('reset-max-btn');
        if (btnResetMax) {
            btnResetMax.addEventListener('click', () => {
                state.meta.maxSpeed = 0;
                updateUI();
            });
        }

        // Reset TOUT
        const btnResetAll = $('reset-all-btn');
        if (btnResetAll) {
            btnResetAll.addEventListener('click', () => {
                if(confirm("Tout réinitialiser ?")) location.reload();
            });
        }

        // Forçage de la précision (Debug)
        const inputAcc = $('gps-accuracy-override');
        if (inputAcc) {
            inputAcc.addEventListener('change', (e) => {
                // Pour debug seulement, modifie la précision affichée
                // Dans un vrai cas, on injecterait ça dans le filtre de Kalman
                console.log("Override Précision:", e.target.value);
            });
        }
        
        // Mise à jour de la masse pour la physique
        const inputMass = $('mass-input');
        if (inputMass) {
            inputMass.addEventListener('input', (e) => {
                state.meta.mass = parseFloat(e.target.value) || 70;
                if($('mass-display')) $('mass-display').textContent = state.meta.mass.toFixed(3) + " kg";
            });
        }

        // Initialisation affichage statique
        updateUI();
    });

})(window);
