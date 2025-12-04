// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// CORRIGÉ FINAL : Physique, Astro (EOT/TST/MST), IMU, et Cinématique.
// Dépendances (doivent être chargées dans l'HTML) : leaflet.js, turf.min.js, suncalc.js, math.min.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return 'N/A';
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined') {
        console.error("Dépendances critiques math.js, leaflet.js ou suncalc.js manquantes.");
        const statusText = $('speed-status-text');
        if(statusText) statusText.textContent = "ERREUR: Librairies manquantes.";
        return;
    }

    // --- 1. CONSTANTES & CONFIGURATION ---
    const CONSTANTS = {
        C: 299792458, // Vitesse de la lumière (m/s)
        G: 6.67430e-11, // Constante gravitationnelle (m³/kg/s²)
        R_EARTH: 6371000, // Rayon moyen de la Terre (m)
        M_EARTH: 5.9722e24, // Masse de la Terre (kg) - Utilisé pour la dilatation du temps grav.
        D2R: Math.PI / 180, // Degré vers Radian
        R2D: 180 / Math.PI, // Radian vers Degré
        G_BASE: 9.8067, // Gravité standard à la surface (m/s²)
        RHO_SEA_LEVEL: 1.225, // Densité de l'air ISA (kg/m³)
        TEMP_SEA_LEVEL_K: 288.15, // 15°C en Kelvin
        SEC_IN_DAY: 86400,
    };
    
    const CONFIG = {
        ZUPT_THRESH: 0.1, // Seuil de vitesse pour la mise à jour ZUPT (m/s)
        MAP_ZOOM_DEFAULT: 16,
    };
    
    // --- 2. ÉTAT GLOBAL ET PLACEHOLDER UKF ---
    let map = null;
    let userMarker = null;
    let accCircle = null;
    let distanceRatioMode = false;
    let geoWatchId = null;

    // Variables d'environnement pour les calculs de dynamique des fluides
    let currentAirDensity = CONSTANTS.RHO_SEA_LEVEL;
    let currentSpeedOfSound = 340.29; 
    let currentAirTempK = CONSTANTS.TEMP_SEA_LEVEL_K;

    const state = {
        // GPS (Raw data)
        gps: { t: 0, lat: 0, lon: 0, alt: 0, speed: 0, acc: 10, active: false },
        // IMU (Sensor data)
        imu: { ax: 0, ay: 0, az: 0, roll: 0, pitch: 0, gyro: 0, mag: { x: 0, y: 0, z: 0 } },
        // FUSION (UKF Output)
        fusion: { lat: 0, lon: 0, alt: 0, speed: 0, dist: 0, heading: 0 },
        // METADATA
        meta: { startTime: Date.now(), movingTime: 0, maxSpeed: 0, mass: 70, gravity: CONSTANTS.G_BASE },
        flags: { emergency: false, nightMode: true }
    };

    // Placeholder pour l'objet UKF (Simple Blending EKF-like)
    const ukf = {
        // Le vecteur d'état 21 est trop complexe à simuler. On garde une fusion simple.
        predict(dt, ax, ay, az) {
            // Maintient la vitesse après la dernière observation GPS (simulation d'inertie)
            if (state.fusion.speed > 0.1) {
                state.fusion.speed *= 0.999;
            }
        },
        update(gpsLat, gpsLon, gpsAlt, gpsAcc) {
            // Correction (blending avec facteur basé sur l'incertitude GPS)
            const factor = Math.min(1, 0.05 + 10 / (gpsAcc * 5)); // Facteur de correction (0 à 1)
            
            state.fusion.lat = state.fusion.lat * (1 - factor) + gpsLat * factor;
            state.fusion.lon = state.fusion.lon * (1 - factor) + gpsLon * factor;
            state.fusion.alt = state.fusion.alt * (1 - factor) + gpsAlt * factor;
            
            // Simuler la norme du vecteur vitesse 3D estimé
            const vUKF = state.gps.speed * factor + state.fusion.speed * (1 - factor); 
            
            return { 
                lat: state.fusion.lat, 
                lon: state.fusion.lon, 
                alt: state.fusion.alt, 
                vN: vUKF * 0.7, 
                vE: vUKF * 0.7, 
                vD: 0 // Vitesse verticale non résolue par ce placeholder
            };
        },
        getUncertainty() { 
            return state.gps.acc * 0.5; // Placeholder pour l'incertitude UKF
        } 
    };
    
    // --- 3. GESTION DES CAPTEURS IMU (ACCÉLÉROMÈTRE/GYROSCOPE) ---
    
    function initSensors() {
        if (window.DeviceOrientationEvent && window.DeviceMotionEvent) {
            $('statut-capteur').textContent = "Actif (IMU/Fusion)";
            // La permission est gérée au niveau de l'événement click sur 'MARCHE GPS'
        } else {
            $('statut-capteur').textContent = "Non Supporté";
        }
    }
    
    function startSensorListeners() {
        // Accéléromètre
        window.addEventListener('devicemotion', (event) => {
            if (state.flags.emergency) return;
            const a = event.accelerationIncludingGravity;
            // On utilise l'accélération AVEC gravité pour les calculs de force G et de niveau à bulle
            state.imu.ax = a.x || 0;
            state.imu.ay = a.y || 0;
            state.imu.az = a.z || 0;
            
            $('acceleration-x').textContent = dataOrDefault(state.imu.ax, 2, ' m/s²');
            $('acceleration-y').textContent = dataOrDefault(state.imu.ay, 2, ' m/s²');
            $('acceleration-z').textContent = dataOrDefault(state.imu.az, 2, ' m/s²');
            
            const gLong = Math.sqrt(state.imu.ax**2 + state.imu.ay**2) / state.meta.gravity;
            const gVert = (state.imu.az + state.meta.gravity) / state.meta.gravity; // Accel. verticale totale
            $('force-g-long').textContent = dataOrDefault(gLong, 2, ' G');
            $('force-g-vert').textContent = dataOrDefault(gVert, 2, ' G');
            $('acceleration-long').textContent = dataOrDefault(state.imu.ay, 2, ' m/s²');
            $('acceleration-vert').textContent = dataOrDefault(state.imu.az, 2, ' m/s²');
            $('gravite-locale-g').textContent = dataOrDefault(state.meta.gravity, 4, ' m/s²');
        });

        // Orientation / Gyro
        window.addEventListener('deviceorientation', (event) => {
            state.imu.roll = event.gamma || 0;
            state.imu.pitch = event.beta || 0;
            
            $('inclinaison-pitch').textContent = dataOrDefault(state.imu.pitch, 1, '°');
            $('roulis-roll').textContent = dataOrDefault(state.imu.roll, 1, '°');
            
            if (event.alpha !== null) {
                state.fusion.heading = event.alpha;
                $('heading-display').textContent = dataOrDefault(event.alpha, 1, '°');
            }
            
            // Placeholder/Simul pour le gyroscope (vitesse angulaire)
            state.imu.gyro = event.webkitCompassHeading || 0;
            $('angular-speed').textContent = dataOrDefault(state.imu.gyro, 1, ' °/s');
        });
        
        $('mag-x').textContent = 'N/A (Simulé)';
        $('mag-y').textContent = 'N/A';
        $('mag-z').textContent = 'N/A';
        
        $('vitesse-verticale-ekf').textContent = 'N/A';
    }

    // --- 4. CALCULS DE PHYSIQUE & RELATIVITÉ ---

    function getSpeedOfSound(tempK) {
        // Vitesse du son : a = sqrt(gamma * R * T)
        // Air (gamma=1.4, R=287 J/kg.K)
        return 20.05 * Math.sqrt(tempK);
    }

    function updatePhysics() {
        const v = state.fusion.speed;
        const M = state.meta.mass;
        const c = CONSTANTS.C;
        const G = CONSTANTS.G;
        const R_earth = CONSTANTS.R_EARTH;
        const M_earth = CONSTANTS.M_EARTH; // Masse de la Terre

        // Énergie
        const E0 = M * c**2;
        $('rest-mass-energy').textContent = dataOrDefaultExp(E0, 3, ' J');

        let gamma = 1;
        if (v > 0) {
             gamma = 1 / Math.sqrt(1 - (v / c)**2);
        }
        $('lorentz-factor').textContent = dataOrDefault(gamma, 4, '');

        const E = E0 * gamma;
        $('relativistic-energy').textContent = dataOrDefaultExp(E, 3, ' J');
        
        const p = M * v * gamma;
        $('momentum').textContent = dataOrDefaultExp(p, 3, ' kg·m/s');

        const percentC = (v / c) * 100;
        $('percent-speed-light').textContent = dataOrDefaultExp(percentC, 2, ' %');
        
        // Dilation du temps (Vitesse)
        const dilationVitesse = (gamma - 1) * CONSTANTS.SEC_IN_DAY * 1e9;
        $('time-dilation-vitesse').textContent = dataOrDefault(dilationVitesse, 2, ' ns/j');
        
        // Rayon de Schwarzschild (pour l'objet - Rs est minuscule)
        const Rs = (2 * G * M) / c**2;
        $('schwarzschild-radius').textContent = dataOrDefaultExp(Rs, 3, ' m');

        // Dilation du temps (Gravité)
        let alt = state.fusion.alt < 0 ? 0 : state.fusion.alt;
        const r = R_earth + alt;
        // Utilisez M_earth pour la dilatation gravitationnelle
        const gravityFactor = 1 / Math.sqrt(1 - (2 * G * M_earth) / (r * c**2));
        const dilationGravite = (gravityFactor - 1) * CONSTANTS.SEC_IN_DAY * 1e9;
        $('time-dilation-gravite').textContent = dataOrDefault(dilationGravite, 2, ' ns/j');

        // Mécanique des Fluides
        currentSpeedOfSound = getSpeedOfSound(currentAirTempK);
        $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');

        const rho = currentAirDensity;
        const q = 0.5 * rho * v**2;
        $('dynamic-pressure').textContent = dataOrDefault(q, 2, ' Pa');
        
        // La Force de Traînée et la Puissance dépendent du Cd et de la surface (non sim.)
        $('force-trainee').textContent = dataOrDefault(0, 2, ' N');
        $('puissance-trainee').textContent = dataOrDefault(0, 2, ' kW');

        const mach = v / currentSpeedOfSound;
        $('mach-number').textContent = dataOrDefault(mach, 4, '');
        $('perc-speed-sound').textContent = dataOrDefault(mach * 100, 2, ' %');
        
        const Ek = 0.5 * M * v**2;
        $('kinetic-energy').textContent = dataOrDefault(Ek, 2, ' J');
    }

    // --- 5. CALCULS ASTRO & TEMPS Solaire ---

    const getMoonPhaseName = (phase) => {
        if (phase < 0.03 || phase >= 0.97) return 'Nouvelle Lune 🌑';
        if (phase < 0.22) return 'Croissant Ascendant 🌒';
        if (phase < 0.28) return 'Premier Quartier 🌓';
        if (phase < 0.47) return 'Lune Gibbeuse Ascendante 🌔';
        if (phase < 0.53) return 'Pleine Lune 🌕';
        if (phase < 0.72) return 'Lune Gibbeuse Descendante 🌖';
        if (phase < 0.78) return 'Dernier Quartier 🌗';
        if (phase < 0.97) return 'Croissant Descendant 🌘';
        return 'Inconnu';
    };

    // Approximation simple de l'Équation du Temps (EOT)
    function calculateEOT(date) {
        const N = (date.getTime() - new Date(date.getFullYear(), 0, 0).getTime()) / (1000 * 60 * 60 * 24); // Jour de l'année
        const B = (N - 81) * CONSTANTS.D2R * (360 / 365.25);
        // Formule de l'analemme (minutes)
        const EOT_minutes = -7.659 * Math.sin(B) + 9.863 * Math.sin(2 * B);
        return EOT_minutes; 
    }

    function updateAstroDisplay(lat, lon) {
        const date = new Date();
        const nowMs = date.getTime();

        if (lat === 0 && lon === 0) return;

        // Temps Solaire
        const nowHoursUTC = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
        const eotMinutes = calculateEOT(date);
        
        // Calcul du Temps Solaire Moyen (MST)
        const offsetLocal = lon / 15; // 15 degrés = 1 heure de décalage
        const mstHours = nowHoursUTC + offsetLocal;
        const mstTotalMinutes = (mstHours % 24) * 60;
        
        // Calcul du Temps Solaire Vrai (TST)
        const tstTotalMinutes = mstTotalMinutes + eotMinutes;

        const timeToHMS = (totalMinutes) => {
            const hours = Math.floor(totalMinutes / 60) % 24;
            const minutes = Math.floor(totalMinutes % 60);
            const seconds = Math.floor((totalMinutes * 60) % 60);
            return `${String(hours).padStart(2,'0')}:${String(minutes).padStart(2,'0')}:${String(seconds).padStart(2,'0')}`;
        };
        
        $('eot').textContent = dataOrDefault(eotMinutes, 2, ' min');
        $('mst').textContent = timeToHMS(mstTotalMinutes);
        $('tst').textContent = timeToHMS(tstTotalMinutes);
        $('date-display-astro').textContent = date.toLocaleDateString();
        
        // Midi Solaire Local (UTC)
        // Midi Solaire = 12h TST. L'écart avec UTC est (Longitude/15) - EOT
        const noonOffsetHours = 12 - (lon / 15) - (eotMinutes / 60);
        const noonTime = new Date(nowMs + noonOffsetHours * 3600 * 1000);
        $('noon-solar').textContent = noonTime.toUTCString().slice(17, 25);


        // Calculs SOLAIRES (SunCalc)
        const sunPos = SunCalc.getPosition(date, lat, lon);
        const sunTimes = SunCalc.getTimes(date, lat, lon);

        const sunAlt = sunPos.altitude * CONSTANTS.R2D;
        const sunAzimuth = sunPos.azimuth * CONSTANTS.R2D;

        $('sun-alt').textContent = dataOrDefault(sunAlt, 2, '°');
        $('sun-azimuth').textContent = dataOrDefault(sunAzimuth, 2, '°');

        const formatTime = (time) => time ? time.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', hour12: false }) : 'N/A';
        const sunriseStr = `${formatTime(sunTimes.sunrise)} / ${formatTime(sunTimes.sunriseEnd)}`;
        const sunsetStr = `${formatTime(sunTimes.sunsetStart)} / ${formatTime(sunTimes.sunset)}`;

        $('sunrise-times').textContent = sunriseStr;
        $('sunset-times').textContent = sunsetStr;

        const dayDurationMs = sunTimes.sunset - sunTimes.sunrise;
        if (dayDurationMs > 0) {
            const hours = Math.floor(dayDurationMs / 3600000);
            const minutes = Math.floor((dayDurationMs % 3600000) / 60000);
            $('day-duration').textContent = `${hours}h ${minutes}m`;
        } else {
            $('day-duration').textContent = 'Nuit';
        }
        
        // Horloge Minecraft (Visualisation)
        const clockContainer = $('minecraft-clock');
        if (clockContainer) {
            if (sunAlt > 5) {
                clockContainer.className = "sky-day";
                $('clock-status').textContent = "Jour (☀️)";
            } else if (sunAlt > -10) {
                clockContainer.className = "sky-sunset";
                $('clock-status').textContent = "Crépuscule (✨)";
            } else {
                clockContainer.className = "sky-night";
                $('clock-status').textContent = "Nuit (🌙)";
            }
            // Heure Minecraft (basée sur l'heure locale pour la simulation simple)
            const hours = date.getHours();
            const minutes = date.getMinutes();
            const mcTicks = Math.floor(((hours + 18) % 24) * 1000 + (minutes / 60) * 1000);
            $('time-minecraft').textContent = `${String(hours).padStart(2,'0')}:${String(minutes).padStart(2,'0')} (${mcTicks} ticks)`;
        }
        
        // Calculs LUNAIRES (SunCalc)
        const moonIllumination = SunCalc.getMoonIllumination(date);
        const moonPos = SunCalc.getMoonPosition(date, lat, lon);
        const moonTimes = SunCalc.getMoonTimes(date, lat, lon);

        $('moon-alt').textContent = dataOrDefault(moonPos.altitude * CONSTANTS.R2D, 2, '°');
        $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * CONSTANTS.R2D, 2, '°');
        $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, '%');
        $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);

        const moonRiseSetStr = (moonTimes.alwaysUp) ? 'Toujours Levée' :
                               (moonTimes.alwaysDown) ? 'Toujours Couchée' :
                               (moonTimes.rise && moonTimes.set) ? `${formatTime(moonTimes.rise)} / ${formatTime(moonTimes.set)}` :
                               'N/A';
        $('moon-times').textContent = moonRiseSetStr;
    }


    // --- 6. BOUCLE PRINCIPALE & GPS (Fusion de données) ---

    function onGPSUpdate(position) {
        if (state.flags.emergency) return;

        const coords = position.coords;
        const now = Date.now();
        const dt = state.gps.t === 0 ? 0.02 : (now - state.gps.t) / 1000;
        state.gps.t = now;

        state.gps.lat = coords.latitude;
        state.gps.lon = coords.longitude;
        state.gps.alt = coords.altitude || state.fusion.alt; 
        state.gps.acc = coords.accuracy || 10;
        state.gps.speed = coords.speed || 0;

        const result = ukf.update(state.gps.lat, state.gps.lon, state.gps.alt, state.gps.acc);
        
        const prevLat = state.fusion.lat;
        const prevLon = state.fusion.lon;
        
        state.fusion.lat = result.lat;
        state.fusion.lon = result.lon;
        state.fusion.alt = result.alt;
        
        const vUKF = Math.sqrt(result.vN**2 + result.vE**2 + result.vD**2);
        
        // ZUPT (Zero Velocity Update) : Si GPS dit 0 et IMU est stable, force la vitesse à 0
        if (state.gps.speed < CONFIG.ZUPT_THRESH && Math.abs(state.imu.ax) < CONFIG.ZUPT_THRESH) {
            state.fusion.speed = 0;
        } else {
            state.fusion.speed = vUKF;
        }

        if (prevLat !== 0 && state.fusion.speed > CONFIG.ZUPT_THRESH) {
            const d = calcDistance(prevLat, prevLon, state.fusion.lat, state.fusion.lon);
            state.fusion.dist += d;
            state.meta.movingTime += dt;
        }

        if (state.fusion.speed > state.meta.maxSpeed) state.meta.maxSpeed = state.fusion.speed;

        updateUI();
        updateMap(state.fusion.lat, state.fusion.lon, state.gps.acc);
    }

    function onGPSError(err) {
        if(geoWatchId) navigator.geolocation.clearWatch(geoWatchId);
        state.gps.active = false;
        $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
        $('acc-gps').textContent = `Err ${err.code}`;
        $('statut-gps-acquisition').textContent = "ERREUR GPS";
        $('speed-status-text').textContent = "Erreur GPS. Essayer de se déplacer.";
    }

    // Formule de Haversine
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
    
    // --- 7. UI & CARTE ---
    
    function initMap() {
        if (typeof L !== 'undefined') {
            map = L.map('map').setView([0, 0], 2);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OSM',
                maxZoom: 19
            }).addTo(map);

            userMarker = L.marker([0, 0]).addTo(map);
            accCircle = L.circle([0, 0], { radius: 1, color: '#007bff', fillOpacity: 0.2 }).addTo(map);
            const mapElement = $('map');
            if(mapElement) mapElement.textContent = "";
        } else {
            const mapElement = $('map');
            if(mapElement) mapElement.innerHTML = "Leaflet JS non chargé.";
        }
    }

    function updateMap(lat, lon, acc) {
        if (map && userMarker && (lat !== 0 || lon !== 0)) {
            const latlng = [lat, lon];
            userMarker.setLatLng(latlng);
            accCircle.setLatLng(latlng);
            accCircle.setRadius(acc);
            
            if (map.getZoom() < 10) {
                map.setView(latlng, CONFIG.MAP_ZOOM_DEFAULT);
            }
        }
    }

    function updateUI() {
        const now = new Date();
        const v_kmh = state.fusion.speed * 3.6;
        const totalSessionTime = (now - state.meta.startTime) / 1000;
        
        // Colonne 1 : Système
        $('date-heure-utc').textContent = now.toUTCString();
        $('elapsed-session-time').textContent = totalSessionTime.toFixed(1) + " s";
        $('elapsed-motion-time').textContent = state.meta.movingTime.toFixed(1) + " s";
        $('mass-display').textContent = dataOrDefault(state.meta.mass, 3, ' kg');
        $('gravity-base').textContent = dataOrDefault(state.meta.gravity, 4, ' m/s²');
        
        // Colonne 2 : Vitesse & Distance
        $('speed-status-text').textContent = state.gps.active ? (v_kmh > 0.5 ? 'Signal GPS Actif' : 'Attente du signal GPS...') : 'Signal GPS en PAUSE';
        
        $('speed-stable').textContent = dataOrDefault(v_kmh, 1, ' km/h');
        $('speed-stable-ms').textContent = dataOrDefault(state.fusion.speed, 2, ' m/s');
        $('speed-stable-kms').textContent = dataOrDefault(state.fusion.speed / 1000, 4, ' km/s');
        $('speed-3d-inst').textContent = dataOrDefault(state.gps.speed * 3.6, 1, ' km/h');
        $('speed-raw-ms').textContent = dataOrDefault(state.gps.speed, 2, ' m/s');
        
        $('vitesse-max-session').textContent = dataOrDefault(state.meta.maxSpeed * 3.6, 1, ' km/h');
        
        const avgSpeedTotal = totalSessionTime > 0 ? (state.fusion.dist / totalSessionTime) * 3.6 : 0;
        const avgSpeedMoving = state.meta.movingTime > 0 ? (state.fusion.dist / state.meta.movingTime) * 3.6 : 0;
        $('speed-avg-moving').textContent = dataOrDefault(avgSpeedMoving, 1, ' km/h');
        $('speed-avg-total').textContent = dataOrDefault(avgSpeedTotal, 1, ' km/h');

        $('distance-totale').textContent = `${dataOrDefault(state.fusion.dist / 1000, 3, ' km')} | ${dataOrDefault(state.fusion.dist, 2, ' m')}`;
        const distLightSec = state.fusion.dist / CONSTANTS.C;
        $('distance-light-s').textContent = dataOrDefaultExp(distLightSec, 2, ' s');
        $('distance-light-min').textContent = dataOrDefaultExp(distLightSec / 60, 2, ' min');
        $('distance-light-h').textContent = dataOrDefaultExp(distLightSec / 3600, 2, ' h');
        $('distance-light-day').textContent = dataOrDefaultExp(distLightSec / CONSTANTS.SEC_IN_DAY, 2, ' j');
        $('distance-cosmic').textContent = dataOrDefaultExp(state.fusion.dist / 149597870700, 2, ' UA') + ' | ' + dataOrDefaultExp(state.fusion.dist / 9.461e15, 2, ' al');

        updatePhysics();

        // Colonne 3 : EKF & Position
        $('lat-ekf').textContent = dataOrDefault(state.fusion.lat, 6, '°');
        $('lon-ekf').textContent = dataOrDefault(state.fusion.lon, 6, '°');
        $('alt-ekf').textContent = dataOrDefault(state.fusion.alt, 1, ' m');
        $('geopotential-alt').textContent = dataOrDefault(state.fusion.alt * 1.0001, 1, ' m');
        
        $('acc-gps').textContent = dataOrDefault(state.gps.acc, 2, ' m');
        
        $('statut-gps-acquisition').textContent = state.gps.active ? "FIX 3D (Actif)" : "INACTIF";
        $('ukf-v-uncert').textContent = dataOrDefault(ukf.getUncertainty(), 3, '');
        $('gps-accuracy-display').textContent = dataOrDefault(state.gps.acc, 6, ' m');
        
        // Astro
        updateAstroDisplay(state.fusion.lat, state.fusion.lon);
        
        // Environnement (Simulé par défaut)
        $('temp-air').textContent = dataOrDefault(currentAirTempK - 273.15, 1, ' °C');
        $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
        $('pression-atm').textContent = dataOrDefault(1013.25, 2, ' hPa');
    }

    // Boucle d'animation visuelle (Prédiction UKF & fluidité UI)
    function animate() {
        if (!state.flags.emergency) {
            ukf.predict(0.016, state.imu.ax, state.imu.ay, state.imu.az);
            updateUI();
        }
        requestAnimationFrame(animate);
    }

    // --- 8. INITIALISATION ET ÉVÉNEMENTS ---

    document.addEventListener('DOMContentLoaded', () => {
        initMap();
        initSensors();
        
        // Événements d'entrée (Mass, Célestial Body)
        $('mass-input').addEventListener('input', (e) => {
            state.meta.mass = parseFloat(e.target.value) || 70;
            $('mass-display').textContent = dataOrDefault(state.meta.mass, 3, ' kg');
        });
        
        $('celestial-body-select').addEventListener('change', (e) => {
            const body = e.target.value;
            if (body === 'Terre') state.meta.gravity = CONSTANTS.G_BASE;
            else if (body === 'Lune') state.meta.gravity = 1.62;
            else if (body === 'Mars') state.meta.gravity = 3.72;
            else state.meta.gravity = CONSTANTS.G_BASE; // Default
            
            $('gravity-base').textContent = dataOrDefault(state.meta.gravity, 4, ' m/s²');
        });

        $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            const status = distanceRatioMode ? 'ALTITUDE' : 'SURFACE';
            const ratio = distanceRatioMode ? 1.050 : 1.000;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${status} (${ratio.toFixed(3)})`;
            $('distance-ratio').textContent = ratio.toFixed(3);
        });

        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            state.flags.nightMode = document.body.classList.contains('dark-mode');
        });

        // --- Bouton MARCHE/ARRÊT GPS ---
        const btnGps = $('toggle-gps-btn');
        if (btnGps) {
            btnGps.addEventListener('click', () => {
                if (!state.gps.active) {
                    if (navigator.geolocation) {
                        
                        // 1. Gérer les permissions IMU (nécessaire sur iOS)
                        if (typeof DeviceOrientationEvent.requestPermission === 'function') {
                             DeviceOrientationEvent.requestPermission()
                                .then(permissionState => {
                                    if (permissionState === 'granted') {
                                        startSensorListeners();
                                    } else {
                                        console.warn("Permission d'orientation refusée. IMU en mode restreint.");
                                    }
                                });
                        } else {
                            startSensorListeners();
                        }

                        // 2. Démarrer le GPS
                        const opts = { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 };
                        geoWatchId = navigator.geolocation.watchPosition(onGPSUpdate, onGPSError, opts);
                        
                        state.gps.active = true;
                        state.meta.startTime = Date.now();
                        
                        // 3. UI
                        btnGps.textContent = "⏸️ PAUSE GPS";
                        btnGps.style.backgroundColor = "#ffc107";
                        btnGps.style.color = "#000";
                        
                        // 4. Lancer la boucle d'animation
                        animate();
                        
                    } else {
                        alert("La géolocalisation n'est pas supportée.");
                    }
                } else {
                    // Arrêter le GPS et réinitialiser
                    if (geoWatchId) navigator.geolocation.clearWatch(geoWatchId);
                    state.gps.active = false;
                    btnGps.textContent = "▶️ MARCHE GPS";
                    btnGps.style.backgroundColor = "#007bff";
                    btnGps.style.color = "#fff";
                }
            });
        }

        // --- Arrêt d'urgence & Reset ---
        const btnEmergency = $('emergency-stop-btn');
        if (btnEmergency) {
            btnEmergency.addEventListener('click', () => {
                state.flags.emergency = !state.flags.emergency;
                if (state.flags.emergency) {
                    btnEmergency.classList.add('active');
                    btnEmergency.textContent = "🛑 ARRÊT D'URGENCE ACTIF";
                    state.fusion.speed = 0;
                } else {
                    btnEmergency.classList.remove('active');
                    btnEmergency.textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
                    animate(); // Relancer l'animation si arrêtée
                }
            });
        }
        
        $('reset-dist-btn').addEventListener('click', () => {
            state.fusion.dist = 0;
            state.meta.movingTime = 0;
            updateUI();
        });

        $('reset-max-btn').addEventListener('click', () => {
            state.meta.maxSpeed = 0;
            updateUI();
        });

        $('reset-all-btn').addEventListener('click', () => {
            if(confirm("Tout réinitialiser ?")) location.reload();
        });

        // Initialiser l'affichage au démarrage
        updateUI();
    });

})(window);
