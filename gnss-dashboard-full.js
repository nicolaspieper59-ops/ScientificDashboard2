// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V11 (UKF 21 ÉTATS)
// CORRECTION : Diagnostic renforcé de l'activation IMU et Fallback Time/Météo
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
    
    // --- VÉRIFICATION DES DÉPENDANCES CRITIQUES (DÉGRADATION GRACIEUSE) ---
    if (typeof math === 'undefined' || typeof ProfessionalUKF === 'undefined') {
         console.error("🔴 ERREUR CRITIQUE: Librairies Math/UKF manquantes. Le filtre UKF est désactivé.");
    }
    if (typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
         console.warn("🟡 AVERTISSEMENT: Cartographie/Astro manquante. Ces sections seront désactivées.");
    }


    // --- CLÉS D'API & ENDPOINTS ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
    const DOM_SLOW_UPDATE_MS = 250; // Taux de rafraîchissement DOM

    
    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const C_L = 299792458; // Vitesse de la lumière
    const G_U = 6.67430e-11; // Constante gravitationnelle
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin (ISA)
    const TEMP_SEA_LEVEL_C = 15.0; // 15°C
    const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
    const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (ISA)
    const R_E_BASE = 6371000; // Rayon terrestre de base (m)

    // --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
    let ukf = (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') ? new ProfessionalUKF() : null;
    let isGpsPaused = false; 
    let isIMUActive = false;
    let netherMode = false;
    let distanceRatioMode = false;
    let currentCelestialBody = 'EARTH';
    let rotationRadius = 100;
    let angularVelocity = 0.0;
    let initTime = Date.now(); 

    // Position initiale par défaut (pour débloquer Astro/Météo)
    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
    let currentMass = 70.0;
    let totalDistance = 0.0;
    let maxSpeed = 0.0;
    let kAlt = 0.0; // Altitude filtrée/estimée
    let gpsWatchID = null;
    let lServH = null; // Heure serveur
    let lLocH = new Date(); // Heure locale au moment de la synchro
    let ntpSyncSuccess = false; 
    
    // Météo et Physique (Fallbacks ISA par défaut)
    let lastKnownWeather = { 
        tempC: TEMP_SEA_LEVEL_C, pressure_hPa: BARO_ALT_REF_HPA, humidity_perc: 50.0,
        air_density: RHO_SEA_LEVEL, tempK: TEMP_SEA_LEVEL_K
    };
    let currentSpeedOfSound = 340.29; 
    let currentAirDensity = RHO_SEA_LEVEL;
    
    let imuAccels = { x: 0, y: 0, z: 0 };
    let imuAngles = { pitch: 0, roll: 0 };
    let lastLat = currentPosition.lat;
    let lastLon = currentPosition.lon;
    let lastIMUTime = 0; 
    let currentUKFReactivity = 'NORMAL'; 

    // --- VARIABLES CARTE LEAFLET ---
    let map = null;
    let marker = null;
    let trackPolyline = null;
    const MAP_INITIAL_ZOOM = 13;


    // =========================================================
    // BLOC 2 : MODÈLES PHYSIQUES ET UTILITAIRES 
    // =========================================================

    function updateCelestialBody(body, alt_m, radius_m = 100, omega_rad_s = 0.0) {
        let G_ACC_NEW = 9.8067; 
        if (body === 'EARTH') {
            G_ACC_NEW = 9.8067 * (R_E_BASE / (R_E_BASE + alt_m)) ** 2;
        } else if (body === 'MOON') {
            G_ACC_NEW = 1.62;
        } else if (body === 'MARS') {
            G_ACC_NEW = 3.72;
        } else if (body === 'ROTATING') {
            const V_ROT = radius_m * omega_rad_s;
            G_ACC_NEW = Math.sqrt(9.8067**2 + (V_ROT**2 / radius_m)**2); 
        }

        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        return { G_ACC_NEW };
    }

    function calculateDistanceRatio(alt_m) {
        if (netherMode) return 8.0; 
        const ratio = (R_E_BASE + alt_m) / R_E_BASE;
        return ratio;
    }
    
    function getSpeedOfSound(T_K) { 
        if (T_K <= 0 || isNaN(T_K)) return 340.29; 
        return 20.0468 * Math.sqrt(T_K); 
    }
    
    function getSchwarzschildRadius(mass_kg) { 
        return (2 * G_U * mass_kg) / (C_L ** 2); 
    }

    function getAstroData(lat, lon, date) {
        if (typeof SunCalc === 'undefined') return {}; 
        const times = SunCalc.getTimes(date, lat, lon);
        const sunPos = SunCalc.getPosition(date, lat, lon);
        const moonIllum = SunCalc.getMoonIllumination(date);
        return { times, sunPos, moonIllum };
    }

    async function syncH() {
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            const data = await response.json();
            lServH = new Date(data.utc_datetime);
            lLocH = new Date();
            ntpSyncSuccess = true;
        } catch (e) {
            // Fallback en cas d'échec
            lServH = new Date(); 
            lLocH = new Date(); 
            ntpSyncSuccess = false;
        }
    }
    
    function getCDate(serverTime, localTimeAtSync) {
        // Retourne l'heure locale si la synchro n'est pas encore faite (ou a échoué)
        if (!serverTime || !localTimeAtSync) return new Date(); 
        const diff_ms = new Date().getTime() - localTimeAtSync.getTime();
        return new Date(serverTime.getTime() + diff_ms);
    }
    
    async function fetchWeather(lat, lon) {
        try {
            const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
            if (!response.ok) throw new Error("Erreur Proxy ou API Météo");
            const data = await response.json();
            const tempK = data.tempC + 273.15;
            
            lastKnownWeather = { 
                tempC: data.tempC, pressure_hPa: data.pressure_hPa, humidity_perc: data.humidity_perc,
                air_density: (data.pressure_hPa * 100) / (287.058 * tempK), tempK: tempK
            };
            currentAirDensity = lastKnownWeather.air_density;
            currentSpeedOfSound = getSpeedOfSound(lastKnownWeather.tempK);

            return lastKnownWeather;
        } catch (e) {
            // Maintient le fallback ISA en cas d'erreur de réseau/API
            console.warn("Échec de la récupération météo. Utilisation des valeurs ISA par défaut.");
            return lastKnownWeather; 
        }
    }

    // =========================================================
    // BLOC 3 : FUSION GNSS / UKF / IMU 
    // =========================================================

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
        
        const timestamp = event.timeStamp || Date.now();
        const dt = (timestamp - lastIMUTime) / 1000 || 0;
        lastIMUTime = timestamp;

        if (ukf && dt > 0 && isIMUActive) {
            // Dead Reckoning
            ukf.predict(dt, imuAccels); 
            
            const ukfState = ukf.getState();
            currentPosition.spd = ukfState.speed;
            currentPosition.alt = ukfState.alt;
            currentPosition.acc = ukfState.kUncert;

            if ($('gps-status-indicator') && !$('gps-status-indicator').textContent.includes('Signal OK')) {
                 $('gps-status-indicator').textContent = `Estimation UKF (IMU) 🟡 Acc: ${dataOrDefault(currentPosition.acc, 2)}m`;
            }
        }
    }

    // 🛑 MISE À JOUR CRITIQUE (V11) : Diagnostic renforcé pour l'activation IMU
    function activateDeviceMotion() {
        const statusEl = $('imu-status');
        if (statusEl) statusEl.textContent = 'Tentative d\'activation... ⏳'; 

        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            // Logique iOS/Safari : Nécessite une permission explicite
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        window.addEventListener('devicemotion', handleDeviceMotion, true);
                        isIMUActive = true;
                        if (statusEl) statusEl.textContent = 'Actif 🟢 (iOS/Permission OK)';
                    } else {
                        isIMUActive = false;
                        if (statusEl) statusEl.textContent = 'Refusé 🔴 (Vérifiez les réglages Safari)';
                    }
                })
                .catch(e => {
                    isIMUActive = false;
                    if (statusEl) statusEl.textContent = 'Erreur 🔴 (API bloquée/Non supportée)';
                });
        } else if (typeof window.DeviceMotionEvent !== 'undefined') {
            // Logique Android/Chrome Standard : Nécessite juste l'appel
            window.addEventListener('devicemotion', handleDeviceMotion, true);
            isIMUActive = true;
            if (statusEl) statusEl.textContent = 'Actif 🟢 (Standard)';
        } else {
            // Logique Fallback : Non supporté
            isIMUActive = false;
            if (statusEl) statusEl.textContent = 'Non supporté 🔴';
        }
    }

    function initGPS() {
        if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);

        const GPS_OPTS = { 
            enableHighAccuracy: true, 
            maximumAge: 3000,     
            timeout: 15000        
        };

        gpsWatchID = navigator.geolocation.watchPosition(
            (position) => {
                if (isGpsPaused) return;

                const gpsData = {
                    lat: position.coords.latitude,
                    lon: position.coords.longitude,
                    alt: position.coords.altitude || 0.0,
                    speed: position.coords.speed || 0.0,
                    acc: position.coords.accuracy,
                    bearing: position.coords.heading || 0.0,
                    timestamp: position.timestamp
                };

                // UKF Update
                if (ukf) {
                    ukf.update(gpsData);
                    const ukfState = ukf.getState();
                    currentPosition.lat = ukfState.lat;
                    currentPosition.lon = ukfState.lon;
                    currentPosition.alt = ukfState.alt;
                    currentPosition.spd = ukfState.speed;
                    currentPosition.acc = ukfState.kUncert;
                } else {
                    currentPosition.lat = gpsData.lat;
                    currentPosition.lon = gpsData.lon;
                    currentPosition.alt = gpsData.alt;
                    currentPosition.spd = gpsData.speed;
                    currentPosition.acc = gpsData.acc;
                }
                
                kAlt = currentPosition.alt; 

                // Logique de distance
                if (typeof turf !== 'undefined' && currentPosition.spd > 0.1) {
                    const from = turf.point([lastLon, lastLat]);
                    const to = turf.point([currentPosition.lon, currentPosition.lat]);
                    const distance2D = turf.distance(from, to, {units: 'meters'});
                    totalDistance += distance2D * (distanceRatioMode ? calculateDistanceRatio(kAlt) : 1.0);
                }
                lastLat = currentPosition.lat;
                lastLon = currentPosition.lon;
                
                const speedKmh = currentPosition.spd * 3.6;
                if (speedKmh > maxSpeed) maxSpeed = speedKmh;
                
                // Mise à jour Météo (toutes les 10 secondes)
                if (Date.now() % 10000 < 500) { 
                    fetchWeather(currentPosition.lat, currentPosition.lon);
                }
                if ($('gps-status-indicator')) $('gps-status-indicator').textContent = `Signal OK 🛰️ (Acc: ${dataOrDefault(currentPosition.acc, 1)}m) 🟢`;
            },
            (error) => {
                if ($('gps-status-indicator')) $('gps-status-indicator').textContent = `ERREUR GPS (${error.code}) 🔴`;
            },
            GPS_OPTS
        );
    }
    

    // =========================================================
    // BLOC 4 & 5 : CARTE (LEAFLET) & MISE À JOUR DU DOM
    // =========================================================
    
    function initMap() {
        if (typeof L === 'undefined' || !$('map-container')) return;
        map = L.map('map-container', { center: [currentPosition.lat, currentPosition.lon], zoom: MAP_INITIAL_ZOOM, attributionControl: false });
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);

        marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);
        trackPolyline = L.polyline([], {color: '#007bff', weight: 3}).addTo(map);
        
        if ($('map-container')) $('map-container').textContent = ''; 
    }
    
    function updateMapDOM() {
        if (!map || !marker || !trackPolyline) {
             if ($('map-container')) $('map-container').textContent = 'Carte Leaflet N/A (librairie manquante)';
             return;
        }
        
        const lat = currentPosition.lat;
        const lon = currentPosition.lon;
        const latlng = [lat, lon];
        
        marker.setLatLng(latlng);
        trackPolyline.addLatLng(latlng);
        
        // Mettre à jour l'affichage de la position estimée (UKF)
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentPosition.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentPosition.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentPosition.alt, 2) + ' m';
        if ($('ukf-uncertainty')) $('ukf-uncertainty').textContent = dataOrDefault(currentPosition.acc, 4); 
    }

    function getMoonPhaseName(phase) {
        if (phase < 0.05 || phase > 0.95) return 'Nouvelle Lune 🌑';
        if (phase < 0.25) return 'Premier Croissant 🌒';
        if (phase < 0.5) return 'Premier Quartier 🌓';
        if (phase < 0.75) return 'Lune Gibbeuse Croissante 🌔';
        if (phase < 0.85) return 'Pleine Lune 🌕';
        return 'Lune Décroissante 🌖';
    }

    function updateAstroDOM(lat, lon) {
        if (typeof getAstroData === 'undefined' || typeof SunCalc === 'undefined') return;
        
        const now = getCDate(lServH, lLocH);
        const astroData = getAstroData(lat, lon, now);
        
        if (astroData.sunPos) {
            const sunAltDeg = astroData.sunPos.altitude * R2D;
            const sunAziDeg = astroData.sunPos.azimuth * R2D;
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunAltDeg, 2) + '°';
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunAziDeg, 2) + '°';
        }
        
        if (astroData.moonIllum) {
            const moonPhaseName = getMoonPhaseName(astroData.moonIllum.phase);
            if ($('moon-phase-name')) $('moon-phase-name').textContent = moonPhaseName;
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moonIllum.fraction * 100, 1) + '%';
        }
    }


    function updateDashboardDOM() {
        // --- MISE À JOUR DES VARIABLES CLÉS (lecture de l'état UKF/IMU) ---
        const speed3D = currentPosition.spd; 
        const speedKmh = speed3D * 3.6;
        const mach = speed3D / currentSpeedOfSound;
        const lightPerc = (speed3D / C_L) * 100;
        const lorentzFactor = 1 / Math.sqrt(1 - (speed3D / C_L) ** 2);
        const restMassEnergy = currentMass * C_L ** 2;
        const schwarzschildRadius = getSchwarzschildRadius(currentMass);
        const elapsedTime = (Date.now() - initTime) / 1000;
        
        // TEMPS (CORRECTION D'ID CRITIQUE 1)
        const now = getCDate(lServH, lLocH);
        if ($('elapsed-session-time')) $('elapsed-session-time').textContent = `${dataOrDefault(elapsedTime, 2)} s`;
    
    // Utilise l'ID de votre HTML: 'local-time-ntp' et 'date-heure-utc'
        if ($('local-time-ntp')) $('local-time-ntp').textContent = now.toLocaleTimeString('fr-FR') + (ntpSyncSuccess ? '' : ' (Local)');
        if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toLocaleDateString('fr-FR') + ' ' + now.toLocaleTimeString('fr-FR') + (ntpSyncSuccess ? ' UTC' : ' LOCAL');


    // MÉTÉO / PHYSIQUE (CORRECTION D'ID CRITIQUE 2, 3 et 4)
    // Utilise les IDs de votre HTML: 'air-temp-c', 'pressure-hpa', 'air-density'
        if ($('air-temp-c')) $('air-temp-c').textContent = dataOrDefault(lastKnownWeather.tempC, 2, ' °C'); 
        if ($('pressure-hpa')) $('pressure-hpa').textContent = dataOrDefault(lastKnownWeather.pressure_hPa, 0, ' hPa'); 
        if ($('air-density')) $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³'); // Anciennement air-density-rho

    // VITESSE/RELATIVITÉ
        if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${dataOrDefault(currentSpeedOfSound, 4)} m/s`;
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(schwarzschildRadius, 4, ' m');
        if ($('vitesse-inst')) $('vitesse-inst').textContent = `${dataOrDefault(speedKmh, 1)} km/h`;
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(lightPerc, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restMassEnergy, 4, ' J');
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(schwarzschildRadius, 4, ' m');
        if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = `${dataOrDefault(speed3D, 2)} m/s`;
        if ($('vitesse-max-session')) $('vitesse-max-session').textContent = `${dataOrDefault(maxSpeed, 1)} km/h`;
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(lightPerc, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restMassEnergy, 4, ' J');
        if ($('dist-total-3d')) $('dist-total-3d').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km | ${dataOrDefault(totalDistance, 2)} m`;
        
        // IMU
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(imuAccels.x, 2) + ' m/s²';
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(imuAccels.y, 2) + ' m/s²';
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(imuAccels.z, 2) + ' m/s²';
        if ($('pitch-imu')) $('pitch-imu').textContent = dataOrDefault(imuAngles.pitch, 1) + '°';
        if ($('roll-imu')) $('roll-imu').textContent = dataOrDefault(imuAngles.roll, 1) + '°';
        // Note: Le statut IMU est mis à jour directement par activateDeviceMotion()
        if ($('ukf-status')) $('ukf-status').textContent = ukf ? 'Actif 🟢' : 'UKF N/A 🔴';


        updateMapDOM();
        updateAstroDOM(currentPosition.lat, currentPosition.lon);
    }


    // =========================================================
    // BLOC 6 : FONCTIONS DE CONTRÔLE ET LISTENERS
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
            // ✅ Activation automatique de l'IMU (Dead Reckoning)
            activateDeviceMotion(); 
            initGPS(); 
            if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'Recherche... 🟡';
        }
    }
    
    function setupEventListeners() {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGpsPause);
        
        if ($('full-reset-btn')) $('full-reset-btn').addEventListener('click', () => { 
            if(confirm("Voulez-vous vraiment TOUT RÉINITIALISER (recharger la page) ?")) { 
                location.reload(); 
            }
        });
        
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
             currentMass = parseFloat(e.target.value) || 70.0;
             if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        
        // Contrôles Corps Céleste/Rotation/Nether/Ratio...
        if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        });
            currentCelestialBody = e.target.value;
            const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        });

        // Contrôles de Rotation
        const updateRotation = () => {
            rotationRadius = parseFloat($('rotation-radius').value) || 100;
            angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
            if (currentCelestialBody === 'ROTATING') {
                const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
                $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
            }
        };
        if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
        if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);

        // Mode Nether
        const netherToggleBtn = $('nether-toggle-btn');
        if (netherToggleBtn) netherToggleBtn.addEventListener('click', () => {
            netherMode = !netherMode;
            netherToggleBtn.textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1) 🌍'}`;
        });
        
        // Rapport Distance
        if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
        });
        
        // Réactivité UKF
        if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => {
            currentUKFReactivity = e.target.value;
            if (ukf && ukf.getProcessNoiseMatrix) {
                ukf.Q_Base = ukf.getProcessNoiseMatrix(currentUKFReactivity); 
            }
        });
    }
    // =========================================================
    // BLOC 7 : INITIALISATION DU SYSTÈME
    // =========================================================

    window.addEventListener('load', () => {
        
        syncH(); 
        initMap(); 
        setupEventListeners();

        // 2. Initialisation UKF (doit se faire après le chargement de math.js)
        if (ukf) { ukf = new ProfessionalUKF(); }
        
        // 3. Force le démarrage du GPS et de l'IMU au chargement (il est initialisé en PAUSE au chargement)
        // La ligne ci-dessous simule le premier clic pour démarrer l'IMU et le GPS
        if (!isGpsPaused && $('toggle-gps-btn')) {
             $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
             activateDeviceMotion(); // Tente d'activer l'IMU (peut échouer si non autorisé)
             initGPS(); // Démarrage du GPS
        }
        
        // Maintien du statut initial IMU si l'activation ci-dessus a échoué silencieusement
        if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';

        // 4. Boucle principale de rafraîchissement
        setInterval(updateDashboardDOM, 250); 
    });

})(window);
