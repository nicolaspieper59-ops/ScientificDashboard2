// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V7 (UKF 21 ÉTATS)
// FUSION DE TOUTES LES MEILLEURES FONCTIONNALITÉS + FIX GPS/IMU ANCIENNE API
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js, lib/ukf-lib.js, lib/astro.js, lib/ephem.js
// - leaflet.js, turf.min.js, suncalc.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {
    
    // --- VÉRIFICATION DES DÉPENDANCES CRITIQUES ---
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
         console.error("🔴 ERREUR CRITIQUE: Dépendances manquantes (math, Leaflet, SunCalc, Turf). Le système est limité.");
    }
    if (typeof ProfessionalUKF === 'undefined') {
        console.error("🔴 ERREUR CRITIQUE: La classe ProfessionalUKF (21 États) est manquante. Charger lib/ukf-lib.js !");
    }

    // --- CLÉS D'API & ENDPOINTS ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
    const DOM_SLOW_UPDATE_MS = 2000;
    
    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const KMH_MS = 3.6;
    const C_L = 299792458;
    const G_U = 6.67430e-11;
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
    const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
    const BARO_ALT_REF_HPA = 1013.25;

    // --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
    let ukf = (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') ? new ProfessionalUKF() : null;
    let isGpsPaused = false; 
    let isIMUActive = false;
    let netherMode = false;
    let distanceRatioMode = false;
    let currentUKFReactivity = 'NORMAL'; 
    let currentCelestialBody = 'EARTH';
    let rotationRadius = 100;
    let angularVelocity = 0.0;
    
    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
    let currentMass = 70.0;
    let totalDistance = 0.0;
    let maxSpeed = 0.0;
    let kAlt = 0.0; 
    let gpsWatchID = null;
    let lServH = null; 
    let lLocH = new Date(); 
    let ntpSyncSuccess = false; 
    let lastKnownWeather = null;
    let isWeatherAPIFailing = true; 
    let currentSpeedOfSound = 340.29; 
    let currentAirDensity = RHO_SEA_LEVEL;
    let imuAccels = { x: 0, y: 0, z: 0 };
    let imuAngles = { pitch: 0, roll: 0 };
    let lastLat = currentPosition.lat;
    let lastLon = currentPosition.lon;
    
    // --- VARIABLES CARTE LEAFLET ---
    let map = null;
    let marker = null;
    let trackPolyline = null;
    const MAP_INITIAL_ZOOM = 13;


    // =========================================================
    // BLOC 2 : MODÈLES PHYSIQUES ET SYSTÈME
    // =========================================================

    function getSpeedOfSound(T_K) { 
        if (T_K <= 0 || isNaN(T_K)) return 340.29;
        return 20.0468 * Math.sqrt(T_K); 
    }
    
    function getSchwarzschildRadius(mass_kg) { 
        return (2 * G_U * mass_kg) / (C_L ** 2); 
    }

    // Assurez-vous que cette fonction est bien dans votre lib/astro.js
    // Si la fonction est manquante, le fallback doit être géré dans updateAstroDOM.
    function getAstroData(lat, lon, date) {
        if (typeof SunCalc === 'undefined') return {}; // Fallback si SunCalc est manquant

        const times = SunCalc.getTimes(date, lat, lon);
        const sunPos = SunCalc.getPosition(date, lat, lon);
        const moonPos = SunCalc.getMoonPosition(date, lat, lon);
        const moonIllum = SunCalc.getMoonIllumination(date);

        return { times, sunPos, moonPos, moonIllum };
    }

    async function syncH() {
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            const data = await response.json();
            lServH = new Date(data.utc_datetime);
            lLocH = new Date();
            ntpSyncSuccess = true;
        } catch (e) {
            lServH = new Date(); 
            lLocH = new Date(); 
            ntpSyncSuccess = false;
        }
    }
    
    function getCDate(serverTime, localTimeAtSync) {
        if (!serverTime || !localTimeAtSync) return new Date();
        const diff_ms = new Date().getTime() - localTimeAtSync.getTime();
        return new Date(serverTime.getTime() + diff_ms);
    }
    
    // ... (fetchWeather conservé)
    async function fetchWeather(lat, lon) {
        try {
            const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
            if (!response.ok) throw new Error("Erreur Proxy ou API Météo");
            const data = await response.json();
            const tempK = data.tempC + 273.15;
            isWeatherAPIFailing = false;
            lastKnownWeather = { 
                tempC: data.tempC, pressure_hPa: data.pressure_hPa, humidity_perc: data.humidity_perc,
                air_density: (data.pressure_hPa * 100) / (287.058 * tempK), tempK: tempK
            };
            return lastKnownWeather;
        } catch (e) {
            isWeatherAPIFailing = true;
            return lastKnownWeather || { 
                tempC: 15.0, pressure_hPa: BARO_ALT_REF_HPA, humidity_perc: 50.0,
                air_density: RHO_SEA_LEVEL, tempK: TEMP_SEA_LEVEL_K
            };
        }
    }

    // =========================================================
    // BLOC 3 : FUSION GNSS / UKF / IMU (avec Fixes)
    // =========================================================

    function initGPS() {
        if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);

        // 🌟 FIX CRITIQUE GPS: Paramètres plus stables et tolérants
        const GPS_OPTS = { 
            enableHighAccuracy: true, 
            maximumAge: 3000,     
            timeout: 15000        
        };

        let lastTimestamp = 0;

        gpsWatchID = navigator.geolocation.watchPosition(
            (position) => {
                if (isGpsPaused) return;

                const timestamp = position.timestamp;
                const dt = (timestamp - lastTimestamp) / 1000 || 0;
                lastTimestamp = timestamp;

                const gpsData = {
                    lat: position.coords.latitude,
                    lon: position.coords.longitude,
                    alt: position.coords.altitude || 0.0,
                    speed: position.coords.speed || 0.0,
                    acc: position.coords.accuracy,
                    bearing: position.coords.heading || 0.0,
                    timestamp: timestamp
                };

                // UKF Predict / Update
                if (ukf) {
                    if (dt > 0 && ukf.x) {
                        ukf.predict(dt); 
                    }
                    ukf.update(gpsData);

                    const ukfState = ukf.getState();
                    currentPosition.lat = ukfState.lat;
                    currentPosition.lon = ukfState.lon;
                    currentPosition.alt = ukfState.alt;
                    currentPosition.spd = ukfState.speed;
                    currentPosition.acc = ukfState.kUncert; // Incertitude UKF
                } else {
                    // Fallback sur données brutes
                    currentPosition.lat = gpsData.lat;
                    currentPosition.lon = gpsData.lon;
                    currentPosition.alt = gpsData.alt;
                    currentPosition.spd = gpsData.speed;
                    currentPosition.acc = gpsData.acc;
                }
                
                kAlt = currentPosition.alt; // Altitude filtrée/brute

                // Logique de calcul de distance et vitesse max (conservée)
                if (lastLat !== null && lastLon !== null && currentPosition.spd > 0.1) {
                    // Calcul de la distance 2D (Turf.js serait utilisé ici si implémenté)
                    const from = turf.point([lastLon, lastLat]);
                    const to = turf.point([currentPosition.lon, currentPosition.lat]);
                    const distance2D = turf.distance(from, to, {units: 'meters'});
                    // Approximation de la distance 3D
                    const distance3D = Math.sqrt(distance2D**2 + (currentPosition.alt - kAlt)**2);
                    totalDistance += distance3D; 
                }
                lastLat = currentPosition.lat;
                lastLon = currentPosition.lon;
                
                const speedKmh = currentPosition.spd * 3.6;
                if (speedKmh > maxSpeed) maxSpeed = speedKmh;
                
                // Mise à jour Météo (toutes les 10 secondes)
                if (Date.now() % 10000 < 500) { 
                    fetchWeather(currentPosition.lat, currentPosition.lon).then(data => {
                        currentAirDensity = data.air_density;
                        currentSpeedOfSound = getSpeedOfSound(data.tempK);
                    });
                }
                if ($('gps-status-indicator')) $('gps-status-indicator').textContent = `Signal OK (Acc: ${dataOrDefault(currentPosition.acc, 1)}m) 🟢`;
            },
            (error) => {
                if ($('gps-status-indicator')) $('gps-status-indicator').textContent = `ERREUR GPS (${error.code}) 🔴`;
                currentPosition.spd = 0.0;
            },
            GPS_OPTS
        );
    }

    function handleDeviceMotion(event) {
        if (event.acceleration) {
            imuAccels.x = event.acceleration.x || 0;
            imuAccels.y = event.acceleration.y || 0;
            imuAccels.z = event.acceleration.z || 0;
        }
        if (event.accelerationIncludingGravity) {
            const ax = event.accelerationIncludingGravity.x;
            const ay = event.accelerationIncludingGravity.y;
            const az = event.accelerationIncludingGravity.z;
            imuAngles.pitch = Math.atan2(ax, Math.sqrt(ay*ay + az*az)) * R2D;
            imuAngles.roll = Math.atan2(ay, Math.sqrt(ax*ax + az*az)) * R2D;
        }
    }

    function activateDeviceMotion() {
        const statusEl = $('imu-status');
        // 🌟 FIX CRITIQUE CAPTEURS: Demande de permission pour iOS/Safari
        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        window.addEventListener('devicemotion', handleDeviceMotion, true);
                        isIMUActive = true;
                        if (statusEl) statusEl.textContent = 'Actif 🟢';
                    } else {
                        if (statusEl) statusEl.textContent = 'Refusé 🔴 (Permission requise)';
                    }
                })
                .catch(e => {
                    if (statusEl) statusEl.textContent = 'Erreur 🔴 (API non fonctionnelle)';
                });
        } else if (typeof window.DeviceMotionEvent !== 'undefined') {
            // Standard device motion API (Android/Desktop)
            window.addEventListener('devicemotion', handleDeviceMotion, true);
            isIMUActive = true;
            if (statusEl) statusEl.textContent = 'Actif 🟢';
        } else {
            if (statusEl) statusEl.textContent = 'Non supporté 🔴';
        }
    }

    // =========================================================
    // BLOC 4 : GESTION DE LA CARTE (LEAFLET)
    // =========================================================
    
    function initMap() {
        if (typeof L === 'undefined' || !$('map-container')) return;

        map = L.map('map-container', { 
            center: [currentPosition.lat, currentPosition.lon], 
            zoom: MAP_INITIAL_ZOOM,
            attributionControl: false // Pour un look plus propre
        });
        
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            // Votre attribution (laissez ceci par courtoisie ou supprimez l'attributiionControl ci-dessus)
            attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OSM</a>'
        }).addTo(map);

        marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);
        trackPolyline = L.polyline([], {color: '#007bff', weight: 3}).addTo(map);
    }
    
    function updateMapDOM() {
        if (!map || !marker || !trackPolyline) return;

        const lat = currentPosition.lat;
        const lon = currentPosition.lon;
        const latlng = [lat, lon];
        
        // Mettre à jour le marqueur
        marker.setLatLng(latlng);
        // Centrer la carte sur la nouvelle position (optionnel, peut être lourd)
        // map.setView(latlng, map.getZoom()); 
        
        // Ajouter à la trace (limiter la taille de la trace si nécessaire)
        trackPolyline.addLatLng(latlng);
        
        // Mise à jour de l'affichage des coordonnées brutes
        if ($('lat-raw')) $('lat-raw').textContent = dataOrDefault(lat, 6);
        if ($('lon-raw')) $('lon-raw').textContent = dataOrDefault(lon, 6);
    }


    // =========================================================
    // BLOC 5 : MISE À JOUR DU DOM (Affichage & Astro)
    // =========================================================

    function updateAstroDOM(lat, lon) {
        if (typeof getAstroData === 'undefined') {
            const errorText = 'API/Lib Astro N/A ❌';
            if ($('sun-alt')) $('sun-alt').textContent = errorText;
            if ($('moon-phase-name')) $('moon-phase-name').textContent = errorText;
            return;
        }
        
        const now = getCDate(lServH, lLocH);
        const astroData = getAstroData(lat, lon, now);
        
        if (astroData.sunPos) {
            const sunAltDeg = astroData.sunPos.altitude * R2D;
            const sunAziDeg = astroData.sunPos.azimuth * R2D;
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunAltDeg, 2) + '°';
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunAziDeg, 2) + '°';
        }
        
        if (astroData.moonPos) {
            const moonAltDeg = astroData.moonPos.altitude * R2D;
            const moonAziDeg = astroData.moonPos.azimuth * R2D;
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonAltDeg, 2) + '°';
            if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonAziDeg, 2) + '°';
        }

        if (astroData.moonIllum) {
            const moonPhaseName = getMoonPhaseName(astroData.moonIllum.phase); // Supposons que cette fonction existe dans astro.js
            if ($('moon-phase-name')) $('moon-phase-name').textContent = moonPhaseName;
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moonIllum.fraction * 100, 1) + '%';
        }

        // Complétez ici avec la logique d'affichage complète pour les heures de lever/coucher, etc.
    }

    // Fonction fictive pour ne pas dépendre entièrement de votre astro.js
    function getMoonPhaseName(phase) {
        if (phase < 0.05 || phase > 0.95) return 'Nouvelle Lune 🌑';
        if (phase < 0.25) return 'Premier Croissant 🌒';
        if (phase < 0.5) return 'Premier Quartier 🌓';
        if (phase < 0.75) return 'Lune Gibbeuse Croissante 🌔';
        if (phase < 0.85) return 'Pleine Lune 🌕';
        return 'Lune Décroissante 🌖';
    }


    function updateDashboardDOM() {
        const speed3D = currentPosition.spd; 
        const speedKmh = speed3D * 3.6;
        const mach = speed3D / currentSpeedOfSound;
        const lightPerc = (speed3D / C_L) * 100;
        const lorentzFactor = 1 / Math.sqrt(1 - (speed3D / C_L) ** 2);
        const restMassEnergy = currentMass * C_L ** 2;
        const schwarzschildRadius = getSchwarzschildRadius(currentMass);
        const elapsedTime = (Date.now() - initTime) / 1000;
        const ukfState = ukf ? ukf.getState() : currentPosition;
        
        // TEMPS
        const now = getCDate(lServH, lLocH);
        if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = `${dataOrDefault(elapsedTime, 2)} s`;
        if ($('local-time-ntp')) {
            let ntpText = now.toLocaleTimeString('fr-FR');
            if (!ntpSyncSuccess) ntpText += ' (Local)'; 
            $('local-time-ntp').textContent = ntpText;
        }
        if ($('date-utc-gmt')) {
             let dateText = now.toLocaleDateString('fr-FR') + ' ' + now.toLocaleTimeString('fr-FR');
             dateText += ntpSyncSuccess ? ' UTC' : ' LOCAL (Échec UTC)';
             $('date-utc-gmt').textContent = dateText;
        }

        // MÉTÉO / PHYSIQUE
        let weatherData = lastKnownWeather;
        if (isWeatherAPIFailing && !weatherData) {
            weatherData = { tempC: 15.0, pressure_hPa: BARO_ALT_REF_HPA, air_density: RHO_SEA_LEVEL };
            if ($('weather-status')) $('weather-status').textContent = 'API ÉCHOUÉE (ISA) 🟡';
        } else if (!isWeatherAPIFailing) {
            if ($('weather-status')) $('weather-status').textContent = 'Actif 🟢';
        }
        if ($('temp-air')) $('temp-air').textContent = dataOrDefault(weatherData ? weatherData.tempC : null, 2, ' °C');
        if ($('pressure-atm')) $('pressure-atm').textContent = dataOrDefault(weatherData ? weatherData.pressure_hPa : null, 0, ' hPa');
        if ($('air-density-rho')) $('air-density-rho').textContent = dataOrDefault(weatherData ? weatherData.air_density : null, 3, ' kg/m³');

        // VITESSE/RELATIVITÉ
        if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${dataOrDefault(currentSpeedOfSound, 4)} m/s`;
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(schwarzschildRadius, 4, ' m');
        if ($('vitesse-inst')) $('vitesse-inst').textContent = `${dataOrDefault(speedKmh, 1)} km/h`;
        if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = `${dataOrDefault(speed3D, 2)} m/s`;
        if ($('vitesse-max-session')) $('vitesse-max-session').textContent = `${dataOrDefault(maxSpeed, 1)} km/h`;
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(lightPerc, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restMassEnergy, 4, ' J');
        if ($('dist-total-3d')) $('dist-total-3d').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km | ${dataOrDefault(totalDistance, 2)} m`;
        
        // POSITION EKF/UKF
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(ukfState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(ukfState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(ukfState.alt, 2) + ' m';
        if ($('gps-accuracy')) $('gps-accuracy').textContent = dataOrDefault(currentPosition.acc, 2) + ' m';
        if ($('ukf-uncertainty')) $('ukf-uncertainty').textContent = dataOrDefault(ukfState.kUncert, 4);
        
        // IMU
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(imuAccels.x, 2) + ' m/s²';
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(imuAccels.y, 2) + ' m/s²';
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(imuAccels.z, 2) + ' m/s²';
        if ($('pitch-imu')) $('pitch-imu').textContent = dataOrDefault(imuAngles.pitch, 1) + '°';
        if ($('roll-imu')) $('roll-imu').textContent = dataOrDefault(imuAngles.roll, 1) + '°';
        if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';

        // Mise à jour de la Carte et de l'Astro
    updateMapDOM();
        updateAstroDOM(currentPosition.lat, currentPosition.lon);
    }


    // =========================================================
// BLOC 6 : FONCTIONS DE CONTRÔLE ET LISTENERS (COMPLET)
// =========================================================
    
    function toggleGpsPause() {
        isGpsPaused = !isGpsPaused;
        const btn = $('toggle-gps-btn');
        if (btn) btn.innerHTML = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
        
        if (isGpsPaused) {
            if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);
            gpsWatchID = null;
            if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'PAUSE ⏸️';
        } else {
            initGPS();
            if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'Recherche... 🟡';
        }
    }
    
    // Fonction d'initialisation de tous les écouteurs d'événements
    function setupEventListeners() {
        // 1. Contrôles de Base et Réinitialisation
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGpsPause);
        if ($('activate-sensors-btn')) $('activate-sensors-btn').addEventListener('click', activateDeviceMotion);
        
        // Bouton de réinitialisation complète
        if ($('full-reset-btn')) $('full-reset-btn').addEventListener('click', () => { 
            if(confirm("Voulez-vous vraiment TOUT RÉINITIALISER (recharger la page) ?")) { 
                location.reload(); 
            }
        });

        // 2. Paramètres Physiques / UKF
        
        // Entrée Masse Utilisateur
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
             currentMass = parseFloat(e.target.value) || 70.0;
             if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });

        // Sélection de Corps Céleste (Gravité)
        if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            // Nécessite la fonction updateCelestialBody définie dans les premiers blocs
            if (typeof updateCelestialBody !== 'undefined') {
                const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
                if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
            }
        });

        // Contrôles de Rotation (pour le mode 'ROTATING')
        const updateRotation = () => {
            rotationRadius = parseFloat($('rotation-radius').value) || 100;
            angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
            if (currentCelestialBody === 'ROTATING' && typeof updateCelestialBody !== 'undefined') {
                const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
                if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
            }
        };
        if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
        if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);
        
        // Mode Nether (Ratio de distance 1:8 pour l'altitude/distance)
        const netherToggleBtn = $('nether-toggle-btn');
        if (netherToggleBtn) netherToggleBtn.addEventListener('click', () => {
            netherMode = !netherMode;
            netherToggleBtn.textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1) 🌍'}`;
        });

        // Réactivité UKF (Matrice de bruit de processus Q)
        if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => {
            currentUKFReactivity = e.target.value;
            if (ukf && ukf.getProcessNoiseMatrix) { // Assure que la méthode existe dans ukf-lib.js
                ukf.Q_Base = ukf.getProcessNoiseMatrix(currentUKFReactivity); 
                console.log(`UKF : Réactivité changée à ${currentUKFReactivity}.`);
            }
        });

        // Bouton "Rapport Distance" (Correction Altitude, si l'altitude est grande)
        if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            // Nécessite la fonction calculateDistanceRatio définie dans les premiers blocs
            const ratio = (distanceRatioMode && typeof calculateDistanceRatio !== 'undefined') ? calculateDistanceRatio(kAlt || 0) : 1.0;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
        });
    }


// =========================================================
// BLOC 7 : INITIALISATION DU SYSTÈME
// =========================================================
    
    window.addEventListener('load', () => {
        
        // 1. Initialisation des systèmes critiques
        syncH(); // Démarrer la synchro NTP
        initMap(); // Initialisation de la carte Leaflet
        initGPS(); // Démarrage du GPS avec options stables
        setupEventListeners(); // Attacher TOUS les contrôles

        // 2. Initialisation de l'état physique par défaut
        if (typeof updateCelestialBody !== 'undefined') {
            // Déclenche une première mise à jour de la gravité (si la fonction existe)
            updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity); 
        }
        
        // 3. Mise à jour de l'affichage initial des statuts
        if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'Recherche... 🟡';
        if ($('ukf-status')) $('ukf-status').textContent = ukf ? 'Actif 🟢' : 'UKF N/A 🔴';
        if ($('toggle-gps-btn') && isGpsPaused === false) $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
        if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';

        // 4. Boucle principale de rafraîchissement (4 fois par seconde)
        setInterval(updateDashboardDOM, 250); 
    });

})(window); // Fermeture de l'IIFE

// --- FIN DU FICHIER ---
