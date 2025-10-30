// ==========================================================
// GNSS SPACETIME DASHBOARD v5.1 - BLOC 1/2
// VARIABLES, CONSTANTES, ET LOGIQUE DE DONNÉES (MODE PROXY SÉCURISÉ)
// ==========================================================

const $ = id => document.getElementById(id); 

// ---------------------- Constantes Fondamentales ----------------------
const DEFAULT_LAT = 48.856614; // Paris
const DEFAULT_LON = 2.3522219; 
const SECONDS_PER_DAY = 86400;
const MC_SECONDS_PER_REAL_SECOND = 72; 
const MIN_SPEED_FOR_MOVEMENT = 0.5; // m/s pour compter le temps de mouvement

const LIGHT_SPEED_MS_VACUUM = 299792458.0; 
const SPEED_OF_SOUND_MS_AIR_0C = 331.3000; 
const SOUND_TEMP_COEFF = 0.606; 
const G_FORCE_MS2 = 9.806650; 
const STANDARD_ATM_PRESSURE_PA = 101325.000; 
const STANDARD_TEMP_C = 20.000;
const STANDARD_TEMP_K = 293.150; 
const REFRACTIVITY_FACTOR_K = 77.6E-6; 

// ---------------------- Variables d'État (Globales) ----------------------
let synchronized_time_ms = null; 
let sessionStartTime = null;     
let lat = DEFAULT_LAT;
let lon = DEFAULT_LON;
let speed_ms = 0; 
let maxSpeed_kmh = 0;
let totalDistance_m = 0;
let totalTimeMoving_s = 0;
let isBrowserGPSActive = false;
let watchID = null;
let isNetherModeActive = false; 
let target = null;
let targetMarker = null;
let map = null;
let currentMarker = null;
let previousPosition = null; 

// Statuts des capteurs
let isDeviceMotionActive = false;
let isAmbientLightActive = false;

// Données Brutes GNSS/Capteurs
let rawData = {
    // GNSS
    altitude: null, accuracy: null, heading: null,
    speed_measured_raw: 0, vertical_speed: null,
    // Météo/Ambiance (API)
    temp_air: null, pressure: null, humidity: null,
    wind_speed: null, cloud_cover: null, uv_index: null,
    // Capteurs d'appareil (Motion/Orientation)
    accel_x: null, accel_y: null, accel_z: null, 
    pitch: null, roll: null,
    // Capteurs d'appareil (Ambiance)
    light_level: null, noise_level: null, magnetic_field: null,
    // Matériaux / Milieu (Non implémenté)
    material_density: null, material_type: null, 
};

// Constantes Physiques Ajustables
let apiEOT = null; 
let currentSpeedOfSound_ms = SPEED_OF_SOUND_MS_AIR_0C + (STANDARD_TEMP_C * SOUND_TEMP_COEFF); 
let currentSpeedOfLight_ms = LIGHT_SPEED_MS_VACUUM / (1.0 + (REFRACTIVITY_FACTOR_K * STANDARD_ATM_PRESSURE_PA / STANDARD_TEMP_K));
let currentG_Force = G_FORCE_MS2; 

// ---------------------- Logique de Correction Physique ----------------------

function adjustPhysicalConstants() {
    // Logique d'ajustement basée sur rawData.temp_air et rawData.pressure
    // ... (Reste inchangé)

    const T = rawData.temp_air; 
    const P = rawData.pressure; 
    const H = rawData.humidity; 

    // --- 1. Vitesse du Son ---
    if (T !== null && P !== null) {
        let v_sound = SPEED_OF_SOUND_MS_AIR_0C + (SOUND_TEMP_COEFF * T);
        if (H !== null && H > 0) { v_sound *= (1 + 0.001 * H); }
        currentSpeedOfSound_ms = v_sound;
    } else {
        currentSpeedOfSound_ms = SPEED_OF_SOUND_MS_AIR_0C + (STANDARD_TEMP_C * SOUND_TEMP_COEFF);
    }
    
    // --- 2. Vitesse de la Lumière (Correction d'indice de réfraction) ---
    if (T !== null && P !== null) {
        const T_Kelvin = T + 273.15;
        const n_air = 1.0 + (REFRACTIVITY_FACTOR_K * P / T_Kelvin); 
        currentSpeedOfLight_ms = LIGHT_SPEED_MS_VACUUM / n_air;
    } else {
        const n_ref = 1.0 + (REFRACTIVITY_FACTOR_K * STANDARD_ATM_PRESSURE_PA / STANDARD_TEMP_K);
        currentSpeedOfLight_ms = LIGHT_SPEED_MS_VACUUM / n_ref; 
    }
}

// ---------------------- Fonction de Récupération Météo (Appel Proxy SÉCURISÉ) ----------------------

async function fetchAndUpdateWeather() {
    // 🚨 ATTENTION : Mettez à jour l'URL avec votre service proxy en ligne (Render/Vercel)
    // Pour les tests locaux, si votre serveur Node est sur le port 3000 :
    const PROXY_BASE_URL = "http://localhost:3000"; 
    // EN PRODUCTION, REMPLACEZ PAR VOTRE URL HTTPS PUBLIQUE (Ex: "https://votre-proxy-meteo.render.com")

    if (lat === DEFAULT_LAT || lon === DEFAULT_LON) {
        console.warn("Position par défaut. Météo indisponible.");
        adjustPhysicalConstants(); 
        return;
    }
    
    const PROXY_URL = `${PROXY_BASE_URL}/api/weather?lat=${lat}&lon=${lon}`;

    try {
        // Le frontend appelle le PROXY (sans clé API)
        const response = await fetch(PROXY_URL);

        if (!response.ok) {
            console.error(`Erreur HTTP lors de l'appel au Proxy: ${response.status} ${response.statusText}`);
            adjustPhysicalConstants(); 
            return;
        }
        
        const data = await response.json();

        // Le format de la réponse (data) est supposé être le même que celui d'OpenWeatherMap
        rawData.temp_air = data.main.temp; 
        rawData.pressure = data.main.pressure * 100; // hPa -> Pa
        rawData.humidity = data.main.humidity / 100; // % -> 0-1
        rawData.wind_speed = data.wind.speed * 3.6; // m/s -> km/h
        rawData.cloud_cover = data.clouds.all; 
        
        adjustPhysicalConstants(); 

    } catch (error) {
        console.warn("Échec de la récupération des données Météo via le Proxy. Assurez-vous que le serveur est démarré.", error);
        adjustPhysicalConstants(); 
    }
}


// ---------------------- Reste des fonctions Capteurs / GPS ----------------------

function handleDeviceMotion(event) {
    // ... (Reste inchangé)
    if (event.accelerationIncludingGravity) {
        rawData.accel_x = event.accelerationIncludingGravity.x;
        rawData.accel_y = event.accelerationIncludingGravity.y;
        rawData.accel_z = event.accelerationIncludingGravity.z;
    } else {
        rawData.accel_x = null; rawData.accel_y = null; rawData.accel_z = null;
    }

    if (event.rotationRate) {
        rawData.pitch = event.rotationRate.beta; 
        rawData.roll = event.rotationRate.gamma; 
    } else if (event.beta !== null) {
        rawData.pitch = event.beta;
        rawData.roll = event.gamma;
    } else {
        rawData.pitch = null; rawData.roll = null;
    }
    isDeviceMotionActive = true;
    $('sensor-status').textContent = `Capteurs: Accél. OK (Pitch/Roll: ${rawData.pitch !== null ? 'OK' : 'N/D'})`;
}

function initSensors() {
    // ... (Reste inchangé)
    if ('DeviceMotionEvent' in window) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion);
                }
            }).catch(console.error);
        } else {
            window.addEventListener('devicemotion', handleDeviceMotion);
        }
    } 

    if ('AmbientLightSensor' in window) {
        try {
            const lightSensor = new AmbientLightSensor({ frequency: 10 }); 
            lightSensor.onreading = (e) => {
                rawData.light_level = lightSensor.illuminance;
                isAmbientLightActive = true;
                $('sensor-status').textContent = `Capteurs: Accél. OK | Lumière OK`;
            };
            lightSensor.start();
        } catch (error) {
            console.warn("Erreur à l'initialisation du Capteur Lumière (HTTPS/localhost requis).");
        }
    } 
    
    if (!isDeviceMotionActive && !isAmbientLightActive) {
        $('sensor-status').textContent = "Capteurs: Non supportés ou accès refusé. (Nécessite HTTPS/localhost).";
    }
}


function toggleGPS() {
    // ... (Reste inchangé)
    if (!navigator.geolocation) {
        alert("Geolocation API n'est pas supportée par ce navigateur.");
        return;
    }

    if (!isBrowserGPSActive) {
        const options = { enableHighAccuracy: true, timeout: 30000, maximumAge: 0 }; 
        watchID = navigator.geolocation.watchPosition(updateGNSSData, handleGPSError, options); 
        isBrowserGPSActive = true;
        $('toggle-gps-btn').textContent = '⏸️ ARRÊT GPS (Browser)';
        $('toggle-gps-btn').classList.add('status-connected');
        $('toggle-gps-btn').classList.remove('status-disconnected');
    } else {
        if (watchID !== null) { navigator.geolocation.clearWatch(watchID); watchID = null; }
        isBrowserGPSActive = false;
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS (Browser)';
        $('toggle-gps-btn').classList.remove('status-connected');
        $('toggle-gps-btn').classList.add('status-disconnected');
        $('precision-api').textContent = 'System Clock (Unverified)';
    }
}

function updateGNSSData(position) {
    // ... (Reste inchangé)
    const coords = position.coords;
    const newLat = coords.latitude;
    const newLon = coords.longitude;
    const newSpeed_ms = (coords.speed !== null && coords.speed >= 0) ? coords.speed : 0;
    const newTimestamp = position.timestamp; 
    
    if (synchronized_time_ms === null) { sessionStartTime = newTimestamp; }
    
    // 1. Mise à jour des métriques
    const speed_kmh = newSpeed_ms * 3.6;
    if (speed_kmh > maxSpeed_kmh) { maxSpeed_kmh = speed_kmh; }
    if (previousPosition !== null) {
        const timeDelta_s = (newTimestamp - previousPosition.timestamp) / 1000;
        const distanceDelta_m = getDistance(previousPosition.lat, previousPosition.lon, newLat, newLon);
        totalDistance_m += distanceDelta_m;
        
        if (newSpeed_ms > MIN_SPEED_FOR_MOVEMENT) { totalTimeMoving_s += timeDelta_s; }
    }

    // 2. Mise à jour des états globaux
    synchronized_time_ms = newTimestamp; 
    lat = newLat;
    lon = newLon;
    speed_ms = newSpeed_ms; 
    
    // 3. Mise à jour des rawData
    rawData.altitude = coords.altitude; 
    rawData.accuracy = coords.accuracy;
    rawData.heading = coords.heading;
    rawData.speed_measured_raw = coords.speed;
    
    // 4. Correction des constantes physiques
    adjustPhysicalConstants(); 
    
    // 5. Mise à jour de la position précédente
    previousPosition = { lat: newLat, lon: newLon, timestamp: newTimestamp };
    
    // 6. Mise à jour de la carte
    updateMapMarker(lat, lon);
}

function handleGPSError(error) {
    // ... (Reste inchangé)
    let errorMessage = '';
    switch(error.code) {
        case error.PERMISSION_DENIED: errorMessage = "Accès à la position refusé par l'utilisateur."; break;
        case error.POSITION_UNAVAILABLE: errorMessage = "Position indisponible (Mauvais signal GPS)."; break;
        case error.TIMEOUT: errorMessage = "Le délai de recherche de position est expiré (Timeout: 30s)."; break;
        default: errorMessage = `Erreur inconnue (${error.code}).`;
    }

    console.error(`Erreur GPS: ${errorMessage}`);
    $('ws-status').textContent = `❌ GPS Erreur: ${errorMessage}`; 
    
    if (watchID !== null) { navigator.geolocation.clearWatch(watchID); watchID = null; }
    isBrowserGPSActive = false;
    $('toggle-gps-btn').textContent = '❌ GPS ERREUR. REDÉMARRER.';
    $('toggle-gps-btn').classList.remove('status-connected');
    $('toggle-gps-btn').classList.add('status-disconnected');
}

function getDistance(lat1, lon1, lat2, lon2) {
    // ... (Reste inchangé)
    const R = 6371e3; const φ1 = lat1 * Math.PI/180; const φ2 = lat2 * Math.PI/180;
    const Δφ = (lat2-lat1) * Math.PI/180;
    const Δλ = (lon2-lon1) * Math.PI/180;
    const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ/2) * Math.sin(Δλ/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    return R * c; 
}

function updateMapMarker(newLat, newLon) {
    // ... (Reste inchangé)
    if (currentMarker) {
        currentMarker.setLatLng([newLat, newLon]);
        map.panTo([newLat, newLon]);
    }
}
// ==========================================================
// GNSS SPACETIME DASHBOARD v5.1 - BLOC 2/2
// AFFICHAGE, ASTRO ET DÉMARRAGE (POINT D'ENTRÉE) (MODE PROXY SÉCURISÉ)
// ==========================================================

// NOTE: Ce bloc utilise les variables et fonctions définies dans le Bloc 1/2.

// ---------------------- Fonctions Utilitaires et Astro ----------------------

function getUTCDate() {
    // ... (Reste inchangé)
    if (synchronized_time_ms === null) return null; 
    const now = new Date(synchronized_time_ms); 
    return new Date(Date.UTC(now.getUTCFullYear(), now.getUTCMonth(), now.getUTCDate(),
        now.getUTCHours(), now.getUTCMinutes(), now.getUTCSeconds(), now.getUTCMilliseconds()));
}

function calculateEOT(d) {
    // ... (Reste inchangé)
    const B = (2 * Math.PI * (d - 81)) / 365.25;
    const EOT_min = (9.87 * Math.sin(2 * B)) - (7.53 * Math.cos(B)) - (1.5 * Math.sin(B));
    return EOT_min; // en minutes
}

function getHeading(lat1, lon1, lat2, lon2) {
    // ... (Reste inchangé)
    const dLon = (lon2 - lon1) * Math.PI / 180;
    lat1 = lat1 * Math.PI / 180; lat2 = lat2 * Math.PI / 180;
    const y = Math.sin(dLon) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
    let brng = Math.atan2(y, x);
    brng = brng * 180 / Math.PI;
    return (brng + 360) % 360;
}

function updateTargetDisplay() {
    // ... (Reste inchangé)
    if (target && lat !== DEFAULT_LAT && lon !== DEFAULT_LON) {
        const distToTarget = getDistance(lat, lon, target.lat, target.lng);
        const headingToTarget = getHeading(lat, lon, target.lat, target.lng);
        $('distance-cible').textContent = `${(distToTarget/1000).toFixed(2)} km`;
        $('target-heading').textContent = `${headingToTarget.toFixed(1)} °`;
    } else {
        $('distance-cible').textContent = 'N/A';
        $('target-heading').textContent = '-- °';
    }
}

const toTimeStr = sec => {
    // ... (Reste inchangé)
    sec = (sec % SECONDS_PER_DAY + SECONDS_PER_DAY) % SECONDS_PER_DAY;
    const h = Math.floor(sec/3600);
    const m = Math.floor((sec%3600)/60);
    const s = Math.floor(sec%60);
    return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`;
};

// ---------------------- 4. Affichage du DOM ----------------------

function updateDisplay(){
    // ... (Reste inchangé)
    if (synchronized_time_ms === null) {
         $('precision-api').textContent = 'Initialisation... (Attente GPS)';
         return; 
    } 
    
    const syncedDate = new Date(synchronized_time_ms);
    const systemTimeNow = Date.now();
    const speed_ms_stable = speed_ms;
    const speed_kmh = speed_ms * 3.6; 
    const isGpsTime = isBrowserGPSActive && Math.abs(synchronized_time_ms - systemTimeNow) < 1000; 

    // --- TEMPS & SESSION ---
    $('local-time').textContent = syncedDate.toLocaleTimeString('fr-FR');
    $('utc-time').textContent = syncedDate.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour: '2-digit', minute: '2-digit', second: '2-digit' });
    const elapsed = (synchronized_time_ms - sessionStartTime)/1000; 
    $('time-elapsed').textContent = elapsed.toFixed(2)+' s';
    $('time-moving').textContent = totalTimeMoving_s.toFixed(2)+' s'; 
    const mcMultiplier = isNetherModeActive ? 8 : 1; 
    $('time-minecraft').textContent = toTimeStr(elapsed * MC_SECONDS_PER_REAL_SECOND * mcMultiplier);
    $('precision-api').textContent = isGpsTime ? `GPS Clock (Synchronized)` : `System Clock (Unverified)`; 
    $('mode-nether').textContent = isNetherModeActive ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)';

    // --- POSITION & CAPTEURS ---
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    const altitude = rawData.altitude;
    $('altitude-gps').textContent = altitude !== null ? `${altitude.toFixed(1)} m (MSL)` : 'N/A'; 
    $('gps-precision').textContent = rawData.accuracy !== null ? rawData.accuracy.toFixed(1) : 'N/A';
    $('heading').textContent = rawData.heading !== null ? `${rawData.heading.toFixed(1)} °` : '--';
    $('is-underground').textContent = (altitude !== null) ? (altitude < -50 ? 'Oui' : 'Non') : 'N/A';
    $('speed-raw-ms').textContent = rawData.speed_measured_raw !== null ? rawData.speed_measured_raw.toFixed(2) : 'N/A';


    // --- VITESSE, DISTANCE & CORRECTIONS ---
    $('speed-stable').textContent = speed_kmh.toFixed(1)+' km/h';
    $('speed-ms-mms').textContent = `${speed_ms_stable.toFixed(2)} m/s | ${(speed_ms_stable*1000).toFixed(0)} mm/s`;
    const avgSpeed = totalTimeMoving_s > 0 ? (totalDistance_m/1000)/(totalTimeMoving_s/3600) : 0;
    $('speed-avg').textContent = avgSpeed.toFixed(5)+' km/h';
    $('speed-max').textContent = maxSpeed_kmh.toFixed(5)+' km/h';
    const distance_km = totalDistance_m / 1000;
    $('distance-total-km').textContent = `${distance_km.toFixed(3)} km | ${totalDistance_m.toFixed(2)} m`;
    const lightTime_s = totalDistance_m / currentSpeedOfLight_ms;
    const lightYear_al = totalDistance_m / (currentSpeedOfLight_ms * 365.25 * SECONDS_PER_DAY);
    $('distance-cosmic').textContent = `${lightTime_s.toExponential(2)} s lumière | ${lightYear_al.toExponential(2)} al`;
    
    $('perc-sound').textContent = ((speed_ms_stable/currentSpeedOfSound_ms)*100).toFixed(2)+' %';
    $('perc-speed-c').textContent = ((speed_ms_stable/currentSpeedOfLight_ms)*100).toExponential(2)+'%';

    // --- PHYSIQUE & CAPTEURS (MISE À JOUR) ---
    const accel_x = rawData.accel_x; const accel_y = rawData.accel_y; const accel_z = rawData.accel_z;

    if (accel_x !== null && accel_y !== null && accel_z !== null) {
        const accel_total = Math.sqrt(accel_x**2 + accel_y**2 + accel_z**2);
        $('accel-total').textContent = accel_total.toFixed(3)+' m/s²';
        $('g-force').textContent = (accel_x/currentG_Force).toFixed(2)+' G';
        $('g-force-centrifugal').textContent = (accel_y/currentG_Force).toFixed(2)+' G';
    } else {
        $('accel-total').textContent = 'N/A'; $('g-force').textContent = 'N/A'; $('g-force-centrifugal').textContent = 'N/A';
    }
    
    // Affichage des données Météo/Capteurs Locaux
    $('vertical-speed').textContent = rawData.vertical_speed !== null ? rawData.vertical_speed.toFixed(2) + ' m/s' : 'N/A'; 
    $('temp-air').textContent = rawData.temp_air !== null ? `${rawData.temp_air.toFixed(1)} °C` : 'N/A';
    $('pressure').textContent = rawData.pressure !== null ? `${(rawData.pressure/100).toFixed(1)} hPa` : 'N/A'; 
    $('humidity').textContent = rawData.humidity !== null ? `${(rawData.humidity*100).toFixed(0)} %` : 'N/A';
    const pitchRoll = (rawData.pitch !== null && rawData.roll !== null) ? `${rawData.pitch.toFixed(1)}° / ${rawData.roll.toFixed(1)}°` : 'N/A';
    $('pitch-roll').textContent = pitchRoll; 
    $('light-level').textContent = rawData.light_level !== null ? `${rawData.light_level} lux` : 'N/A';
    $('wind-speed').textContent = rawData.wind_speed !== null ? `${rawData.wind_speed.toFixed(1)} km/h` : 'N/A';
    $('cloud-cover').textContent = rawData.cloud_cover !== null ? `${rawData.cloud_cover} %` : 'N/A';
    
    // Champs non implémentés ou par défaut
    $('uv-index').textContent = rawData.uv_index !== null ? `${rawData.uv_index}` : 'N/A';
    $('magnetic-field').textContent = 'N/A (API non impl.)';
    $('noise-level').textContent = 'N/A (API non impl.)';
    $('boiling-point').textContent = rawData.pressure !== null ? 'Calculé...' : 'N/A'; 
}


// ---------------------- 5. Calculs Astronomiques ----------------------

function calculateSolarTimes() {
    // ... (Reste inchangé)
    if (synchronized_time_ms === null || lat === DEFAULT_LAT || lon === DEFAULT_LON) return; 
    
    const SunCalc = window.SunCalc;
    const nowUTC = getUTCDate();
    const lonDeg = lon;
    
    const start = new Date(nowUTC.getFullYear(), 0, 0);
    const diff = nowUTC - start;
    const dayOfYear = Math.floor(diff / (1000 * 60 * 60 * 24));
    
    const EOT_minutes = apiEOT !== null ? apiEOT : calculateEOT(dayOfYear);
    $('eot').textContent = `${EOT_minutes.toFixed(2)} min (Calculé)`; 
    
    const UTCSec = (nowUTC.getUTCHours() * 3600) + (nowUTC.getUTCMinutes() * 60) + nowUTC.getUTCSeconds() + (nowUTC.getUTCMilliseconds() / 1000);
    const lonCorrectionSec = lonDeg * 240; 
    
    let LSM_Sec = (UTCSec + lonCorrectionSec);
    LSM_Sec = LSM_Sec % SECONDS_PER_DAY;
    
    let TST_Sec = (LSM_Sec - (EOT_minutes * 60)); 
    TST_Sec = (TST_Sec % SECONDS_PER_DAY + SECONDS_PER_DAY) % SECONDS_PER_DAY; 

    // --- AFFICHAGE ASTRONOMIE ---
    $('date-astro').textContent = nowUTC.toLocaleDateString('fr-FR');
    $('time-solar-true').textContent = toTimeStr(TST_Sec); 

    const times = SunCalc.getTimes(nowUTC, lat, lon);
    const sunPos = SunCalc.getPosition(nowUTC, lat, lon);
    const dateToTimeStrFn = (dateObj) => dateObj ? dateObj.toLocaleTimeString('fr-FR') : 'N/D';

    $('solar-culmination').textContent = dateToTimeStrFn(times.solarNoon); 
    $('sun-elevation').textContent = `${(sunPos.altitude*180/Math.PI).toFixed(2)} °`;
    
    $('lunar-phase-perc').textContent = `${(SunCalc.getMoonIllumination(nowUTC).fraction*100).toFixed(1)} %`;
}


// ---------------------- 6. Initialisation et Démarrage ----------------------

function initControls() {
    // ... (Reste inchangé)
    $('toggle-gps-btn').addEventListener('click', toggleGPS); 
    $('toggle-mode-btn').addEventListener('click', () => document.body.classList.toggle('night-mode'));
    
    const resetMetrics = () => {
        totalDistance_m=0; maxSpeed_kmh=0; totalTimeMoving_s=0; 
        previousPosition=null;
        console.log('Métriques réinitialisées.');
    };
    
    $('reset-all-btn').addEventListener('click', () => {
        if (synchronized_time_ms !== null) sessionStartTime = synchronized_time_ms;
        resetMetrics();
    });
    
    $('reset-dist-btn').addEventListener('click', () => {
        totalDistance_m=0; totalTimeMoving_s=0; 
    });

    $('reset-max-btn').addEventListener('click', () => {
        maxSpeed_kmh=0;
    });

    $('emergency-stop-btn').addEventListener('click', e => {
        const btn = e.target;
        if(btn.textContent.includes('INACTIF')) {
            btn.textContent='🛑 Arrêt d\'urgence: ACTIF'; btn.classList.add('active'); speed_ms = 0; 
        }
        else {
            btn.textContent='🛑 Arrêt d\'urgence: INACTIF'; btn.classList.remove('active');
        }
    });

    $('set-target-btn').addEventListener('click', ()=>{
        if (lat === DEFAULT_LAT || lon === DEFAULT_LON) {
            alert('Veuillez activer le GPS et obtenir une position réelle avant de définir une cible.');
            return;
        }
        alert('Cliquez sur la carte pour définir la cible.');
        if (map) {
            map.once('click', e=>{
                target = e.latlng;
                if(targetMarker) map.removeLayer(targetMarker);
                if (L) {
                    targetMarker=L.marker(target, {icon: L.divIcon({className: 'target-icon', html: '🎯', iconSize: [24, 24]})}).addTo(map);
                }
                updateTargetDisplay();
            });
        }
    });

    $('nether-toggle-btn').addEventListener('click', () => { isNetherModeActive = !isNetherModeActive; });
}

function initMap() {
    // ... (Reste inchangé)
    const mapContainer = $('map-container');
    if (mapContainer && window.L) {
        map = L.map('map-container').setView([lat, lon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
        currentMarker = L.marker([lat, lon]).addTo(map).bindPopup("Position Actuelle").openPopup();
    } else {
        console.error("Erreur: Le conteneur de carte ou la bibliothèque Leaflet n'est pas disponible.");
    }
}

document.addEventListener('DOMContentLoaded', () => {
    initMap();
    initControls();
    
    // ⭐ Initialisation des capteurs (nécessite HTTPS/localhost)
    initSensors(); 

    // Boucles de rafraîchissement
    setInterval(updateDisplay, 100); 
    setInterval(calculateSolarTimes, 1000); 
    
    // Démarre la boucle de récupération Météo (toutes les 5 minutes)
    // NOTE: C'est l'appel au proxy sécurisé !
    setInterval(fetchAndUpdateWeather, 300000); 
    
    adjustPhysicalConstants(); 
    toggleGPS();
});
