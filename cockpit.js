// --- ENREGISTREMENT DU SERVICE WORKER ---
if ('serviceWorker' in navigator) {
    window.addEventListener('load', function() {
        navigator.serviceWorker.register('/sw.js').then(function(registration) {
            console.log('ServiceWorker enregistré avec le scope:', registration.scope);
        }, function(err) {
            console.log('ServiceWorker erreur:', err);
        });
    });
}

// --- CONSTANTES GLOBALES ET ASTRONOMIQUES ---
const C_LIGHT_MS = 299792458;  // Vitesse de la lumière en m/s
const C_SON_MS_SEA_LEVEL = 343; // Vitesse du son en m/s
const METER_TO_FEET = 3.28084;  
const EARTH_ROTATION_RATE = 15; // 15 degrés par heure (Rotation de la Terre)
const SYNODIC_MONTH = 29.53058867; // Durée moyenne d'un cycle lunaire

// *** SEUILS DE FILTRAGE CRITIQUES (VALEURS ORIGINALES RESTAURÉES) ***
const ALTITUDE_ACCURACY_THRESHOLD = 10.0; 
const HORIZONTAL_ACCURACY_THRESHOLD = 50.0; 

// --- Variables de contrôle et de navigation ---
let intervalId = null;
let timeElapsed = 0; 

let currentSpeedMS = 0; // Vitesse Horizontale (Ground Speed) en m/s
let maxSpeedKPH = 0;
let distanceTraveled = 0; // en km

let currentLat = null;
let currentLon = null;
let currentAlt = null;
let currentAccuracy = 9999; // Précision horizontale (mètres)
let currentAltAccuracy = 9999; // Précision verticale (mètres)

let previousAltitude = null; 
let verticalSpeedMS = 0; // Vitesse Verticale (Vz) en m/s
let previousTimestamp = null; // Pour le calcul temporel précis
let gpsWatchId = null;


// --- FONCTIONS UTILITAIRES DE TEMPS & ASTRONOMIE ---

function getAtomicTimeUTC() {
    const now = new Date();
    const totalHours = now.getUTCHours() + (now.getUTCMinutes() / 60) + (now.getUTCSeconds() / 3600) + (now.getUTCMilliseconds() / 3600000);
    return totalHours;
}

function updateAtomicTime() {
    const now = new Date();
    const utcHours = String(now.getUTCHours()).padStart(2, '0');
    const utcMinutes = String(now.getUTCMinutes()).padStart(2, '0');
    const utcSeconds = String(now.getUTCSeconds()).padStart(2, '0');
    const utcMilliseconds = String(Math.floor(now.getUTCMilliseconds() / 10)).padStart(2, '0'); 
    
    const el = document.getElementById('atomic-time');
    if (el) el.textContent = `${utcHours}:${utcMinutes}:${utcSeconds}.${utcMilliseconds} UTC`;
}

function calculateLunarData() { 
    const newMoonEpoch = new Date('2000-01-06T18:14:00Z');
    const now = new Date();
    const totalDays = (now.getTime() - newMoonEpoch.getTime()) / (1000 * 60 * 60 * 24);
    let daysIntoCycle = totalDays % SYNODIC_MONTH;
    if (daysIntoCycle < 0) {
        daysIntoCycle += SYNODIC_MONTH;
    }
    const phasePercent = ((1 - Math.cos(2 * Math.PI * daysIntoCycle / SYNODIC_MONTH)) / 2) * 100;
    const magnitude = (phasePercent / 100) * (0.5) + 0.5;

    return { 
        phasePercent: phasePercent.toFixed(1), 
        magnitude: magnitude.toFixed(1) 
    };
}


function calculateLocalSolarTime(longitude) {
    const now = new Date();
    const utcTotalHours = getAtomicTimeUTC(); 
    const longitudeOffsetHours = longitude / EARTH_ROTATION_RATE; 

    const yearStart = new Date(now.getUTCFullYear(), 0, 1);
    const dayOfYear = (now.getTime() - yearStart.getTime()) / (1000 * 60 * 60 * 24);
    
    const B = (2 * Math.PI * (dayOfYear - 81) / 365.25); 

    const ECCENTRICITY_APPROX_FACTOR = -7.38217; 
    const OBLIQUITY_APPROX_FACTOR = 9.869;
    const EARTH_ORBITAL_ECCENTRICITY = 0.0167; 
    
    const eccSeconds = (ECCENTRICITY_APPROX_FACTOR * EARTH_ORBITAL_ECCENTRICITY * Math.sin(B)) * 60; 
    const oblSeconds = (OBLIQUITY_APPROX_FACTOR * Math.sin(2 * B)) * 60; 
    
    const edtSeconds = eccSeconds + oblSeconds; 

    let hsmTotalHours = utcTotalHours + longitudeOffsetHours;
    hsmTotalHours = (hsmTotalHours % 24 + 24) % 24; 
    const hsmSecondsTotal = hsmTotalHours * 3600;
    const hsmTime = formatSecondsToTime(hsmSecondsTotal);

    let hsvTotalSeconds = hsmTotalHours * 3600 + edtSeconds;
    hsvTotalSeconds = (hsvTotalSeconds % 86400 + 86400) % 86400;
    const hsvTime = formatSecondsToTime(hsvTotalSeconds);

    const noonUTCSec = 12 * 3600; 
    const longitudeOffsetSeconds = longitudeOffsetHours * 3600;
    let culmTotalSeconds = noonUTCSec - longitudeOffsetSeconds - edtSeconds;
    const localOffset = now.getTimezoneOffset() * 60; 
    let culmLocalSeconds = culmTotalSeconds - localOffset; 
    culmLocalSeconds = (culmLocalSeconds % 86400 + 86400) % 86400;
    const culmTime = formatSecondsToTime(culmLocalSeconds);

    const solarDayDurationSeconds = 86400 + edtSeconds * 0.005; 
    const solarDayDuration = formatSecondsToTime(solarDayDurationSeconds);

    return { 
        hsmTime, 
        hsvTime, 
        edtSeconds: edtSeconds.toFixed(8), 
        culmTime,
        solarDayDuration
    };
}

function formatSecondsToTime(totalSeconds) {
    const hours = Math.floor(totalSeconds / 3600);
    const minutes = Math.floor((totalSeconds % 3600) / 60);
    const seconds = Math.floor(totalSeconds % 60);
    return `${String(hours % 24).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

// --- GESTION DES CAPTEURS ET GPS ---

function startBubbleLevel() { 
    const el = document.getElementById('niveau-bulle');
    if (!('DeviceOrientationEvent' in window)) {
        if (el) el.textContent = '--° (N/A)';
        return;
    }

    function enableOrientation() {
        window.addEventListener('deviceorientation', function(event) {
            const tiltX = event.gamma; 
            const tiltY = event.beta;
            if (tiltX !== null && tiltY !== null) {
                const resultX = tiltX.toFixed(1); 
                const resultY = tiltY.toFixed(1); 
                if (el) el.textContent = `Lat: ${resultX}° | Prof: ${resultY}°`;
            }
        }, true);
    }

    // iOS requires explicit permission (must be in a user event)
    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
        // Show a button to ask for permission if not already present
        let btn = document.getElementById('sensor-btn');
        if (!btn) {
            btn = document.createElement('button');
            btn.id = 'sensor-btn';
            btn.textContent = 'Activer capteurs';
            btn.style.margin = "8px";
            if (el && el.parentElement) el.parentElement.appendChild(btn);
        }
        btn.onclick = () => {
            DeviceOrientationEvent.requestPermission().then(response => {
                if (response === 'granted') {
                    enableOrientation();
                    btn.style.display = 'none';
                } else {
                    if (el) el.textContent = '--° (Permission refusée)';
                }
            }).catch(() => {
                if (el) el.textContent = '--° (Erreur permission)';
            });
        };
    } else {
        enableOrientation();
    }
}


function getGeoLocation() {
    const gpsStatusEl = document.getElementById('gps-status');
    if (!("geolocation" in navigator)) {
        if (gpsStatusEl) gpsStatusEl.textContent = 'N/A (Non supporté)';
        return;
    }
    
    if (gpsWatchId) {
        navigator.geolocation.clearWatch(gpsWatchId);
    }
    
    gpsWatchId = navigator.geolocation.watchPosition(
        (position) => {
            const coords = position.coords;
            const currentTimestampMs = position.timestamp;
            
            currentAccuracy = coords.accuracy; 
            currentAltAccuracy = coords.altitudeAccuracy !== null ? coords.altitudeAccuracy : currentAccuracy * 1.5;

            if (coords.speed !== null && coords.speed !== undefined && currentAccuracy < HORIZONTAL_ACCURACY_THRESHOLD) {
                currentSpeedMS = Math.max(0, coords.speed); 
            } else {
                currentSpeedMS = 0; 
            }
            
            const newAltitude = coords.altitude;
            
            if (previousAltitude !== null && newAltitude !== null && previousTimestamp !== null) {
                const timeDiffSeconds = (currentTimestampMs - previousTimestamp) / 1000;
                
                if (timeDiffSeconds > 0) {
                    verticalSpeedMS = (newAltitude - previousAltitude) / timeDiffSeconds;
                } else {
                    verticalSpeedMS = 0;
                }
                
                if (currentAltAccuracy > ALTITUDE_ACCURACY_THRESHOLD) {
                    verticalSpeedMS = 0;
                    const vvert = document.getElementById('vitesse-vert');
                    if (vvert) vvert.classList.add('warning');
                } else {
                    const vvert = document.getElementById('vitesse-vert');
                    if (vvert) vvert.classList.remove('warning');
                }
                
            } else {
                verticalSpeedMS = 0;
            }
            
            previousAltitude = newAltitude;
            previousTimestamp = currentTimestampMs;

            currentLat = coords.latitude;
            currentLon = coords.longitude;
            currentAlt = newAltitude; 

            const latEl = document.getElementById('latitude');
            if (latEl) latEl.textContent = currentLat !== null ? currentLat.toFixed(6) : '--';

            const lonEl = document.getElementById('longitude');
            if (lonEl) lonEl.textContent = currentLon !== null ? currentLon.toFixed(6) : '--';
            
            const altMeters = currentAlt !== null ? currentAlt.toFixed(1) : '--';
            const altFeet = currentAlt !== null ? (currentAlt * METER_TO_FEET).toFixed(0) : '--';

            const altEl = document.getElementById('altitude');
            if (altEl) altEl.textContent = `${altMeters} m`;

            const altFtEl = document.getElementById('altitude-ft');
            if (altFtEl) altFtEl.textContent = `${altFeet} ft`; 
            
            const vvertEl = document.getElementById('vitesse-vert');
            if (vvertEl) vvertEl.textContent = `${verticalSpeedMS.toFixed(2)} m/s`;
            
            const precMEl = document.getElementById('precision-m');
            if (precMEl) precMEl.textContent = `${currentAccuracy.toFixed(1)} m`;

            const precGpsEl = document.getElementById('prec-gps');
            if (precGpsEl) precGpsEl.textContent = `${(Math.max(0, 100 - (currentAccuracy / 10) * 10)).toFixed(0)} %`; 
            
            if (gpsStatusEl) {
                gpsStatusEl.textContent = 'ACTIF';
                gpsStatusEl.classList.remove('warning');
            }
            
            updateCelestialData(); 
        },
        (error) => {
            currentLat = null; currentLon = null; currentAlt = null; previousAltitude = null; 
            verticalSpeedMS = 0; currentSpeedMS = 0; 
            currentAccuracy = 9999; currentAltAccuracy = 9999; previousTimestamp = null;
            
            const vvertEl = document.getElementById('vitesse-vert');
            if (vvertEl) {
                vvertEl.textContent = '0.00 m/s';
                vvertEl.classList.add('warning');
            }
            if (gpsStatusEl) {
                gpsStatusEl.textContent = `Erreur GPS: ${error.message} - Arrêté`;
                gpsStatusEl.classList.add('warning');
            }
            updateCelestialData(); 
        },
        { enableHighAccuracy: true, timeout: 10000, maximumAge: 0 } 
    );
}

function updateNavigationData() {
    if (!intervalId) return;
    
    const totalGroundSpeedSq = Math.pow(currentSpeedMS, 2) + Math.pow(verticalSpeedMS, 2);
    const TOTAL_GROUND_SPEED_MS = Math.sqrt(totalGroundSpeedSq);
    
    distanceTraveled += (TOTAL_GROUND_SPEED_MS / 1000); 
    
    const V_KPH_INST = Math.max(0, currentSpeedMS * 3.6); 
    const V_KPH_3D = Math.max(0, TOTAL_GROUND_SPEED_MS * 3.6); 
    
    if (V_KPH_INST > maxSpeedKPH) {
        maxSpeedKPH = V_KPH_INST;
    }
    
    const V_MMS = Math.max(0, TOTAL_GROUND_SPEED_MS * 1000);
    const V_LIGHT_RATIO = Math.max(0, TOTAL_GROUND_SPEED_MS / C_LIGHT_MS);
    const MACH_NUMBER = Math.max(0, TOTAL_GROUND_SPEED_MS / C_SON_MS_SEA_LEVEL);
    const AVG_SPEED_KPH = timeElapsed > 0 ? distanceTraveled / (timeElapsed / 3600) : 0;
    
    const avgSpeedKPH = Math.max(0, AVG_SPEED_KPH); 
    const DISTANCE_M = distanceTraveled * 1000;
    const DISTANCE_SL = DISTANCE_M / C_LIGHT_MS; 
    
    const timeEl = document.getElementById('time-s');
    if (timeEl) timeEl.textContent = `${(timeElapsed + 1).toFixed(0)} s`; 

    const vinstEl = document.getElementById('vitesse-inst');
    if (vinstEl) vinstEl.textContent = `${V_KPH_INST.toFixed(2)} km/h`; 

    const v3dEl = document.getElementById('vitesse-3d');
    if (v3dEl) v3dEl.textContent = `${V_KPH_3D.toFixed(2)} km/h`; 

    const vmoyEl = document.getElementById('vitesse-moy');
    if (vmoyEl) vmoyEl.textContent = `${avgSpeedKPH.toFixed(2)} km/h`;

    const vmaxEl = document.getElementById('vitesse-max');
    if (vmaxEl) vmaxEl.textContent = `${maxSpeedKPH.toFixed(2)} km/h`;

    const vmsEl = document.getElementById('vitesse-ms');
    if (vmsEl) vmsEl.textContent = `${Math.max(0, TOTAL_GROUND_SPEED_MS).toFixed(2)} m/s`; 

    const vmmsEl = document.getElementById('vitesse-mms');
    if (vmmsEl) vmmsEl.textContent = `${V_MMS.toFixed(0)} mm/s`;

    const machEl = document.getElementById('mach-number');
    if (machEl) machEl.textContent = `${MACH_NUMBER.toFixed(3)}`; 

    const plEl = document.getElementById('pourcent-lumiere');
    if (plEl) plEl.textContent = `${(V_LIGHT_RATIO * 100).toFixed(8)}%`;

    const distkmEl = document.getElementById('distance-km');
    if (distkmEl) distkmEl.textContent = `${distanceTraveled.toFixed(3)} km`;

    const distmEl = document.getElementById('distance-m');
    if (distmEl) distmEl.textContent = `${DISTANCE_M.toFixed(1)} m`;

    const dslEl = document.getElementById('distance-sl');
    if (dslEl) dslEl.textContent = `${DISTANCE_SL.toPrecision(4)} s lumière`;
    
    timeElapsed++; 
}

function updateCelestialData() {
    const now = new Date(); 
    
    let hsmTime = '--:--:--';
    let hsvTime = '--:--:--';
    let edtSeconds = '--';
    let culmTime = '--:--:--';
    let solarDayDuration = '--:--:--';

    const lunarData = calculateLunarData();
    
    if (currentLon !== null) {
        const solarTimes = calculateLocalSolarTime(currentLon);
        hsmTime = solarTimes.hsmTime;
        hsvTime = solarTimes.hsvTime;
        edtSeconds = solarTimes.edtSeconds;
        culmTime = solarTimes.culmTime;
        solarDayDuration = solarTimes.solarDayDuration;
    }

    const culmEl = document.getElementById('culm-soleil');
    if (culmEl) culmEl.textContent = culmTime;

    const hsmEl = document.getElementById('hsm');
    if (hsmEl) hsmEl.textContent = hsmTime;

    const hsvEl = document.getElementById('hsv');
    if (hsvEl) hsvEl.textContent = hsvTime;

    const edtEl = document.getElementById('edt');
    if (edtEl) edtEl.textContent = `${edtSeconds} s`; 

    const sddEl = document.getElementById('solar-day-duration');
    if (sddEl) sddEl.textContent = solarDayDuration;

    const phaseEl = document.getElementById('phase-lune');
    if (phaseEl) phaseEl.textContent = `${lunarData.phasePercent}%`;

    const magEl = document.getElementById('mag-lune');
    if (magEl) magEl.textContent = lunarData.magnitude;
    
    const totalSecondsInDay = (now.getHours() * 3600) + (now.getMinutes() * 60) + now.getSeconds();
    const minecraftTimeRatio = 1200 / 86400; 
    const minecraftSeconds = totalSecondsInDay * minecraftTimeRatio;
    const mcHours = Math.floor(minecraftSeconds / 50); 
    const mcMinutes = Math.floor((minecraftSeconds % 50) * 1.2); 
    const mcTimeStr = `${String(mcHours % 24).padStart(2, '0')}:${String(mcMinutes % 60).padStart(2, '0')}:00`;
    const mcEl = document.getElementById('horloge-minecraft');
    if (mcEl) mcEl.textContent = mcTimeStr;

    const sousEl = document.getElementById('souterrain');
    if (sousEl) sousEl.textContent = currentAlt === null ? 'Oui' : 'Non'; 
}

function toggleMovement(start) {
    const gpsStatusEl = document.getElementById('gps-status');
    const startBtn = document.getElementById('start-btn');
    const stopBtn = document.getElementById('stop-btn');
    if (start) {
        getGeoLocation(); 
        intervalId = setInterval(() => {
            updateCelestialData(); 
            updateNavigationData(); 
        }, 1000);
        if (gpsStatusEl) {
            gpsStatusEl.textContent = 'ACTIF (Acquisition Vitesse)';
            gpsStatusEl.classList.add('warning');
        }
    } else {
        clearInterval(intervalId);
        intervalId = null;
        
        if (gpsWatchId) {
            navigator.geolocation.clearWatch(gpsWatchId);
            gpsWatchId = null;
        }
        if (gpsStatusEl) {
            gpsStatusEl.textContent = 'Arrêté';
            gpsStatusEl.classList.remove('warning');
        }
        currentSpeedMS = 0; 
    }
    if (startBtn) startBtn.disabled = start;
    if (stopBtn) stopBtn.disabled = !start;
}

function resetData() { 
    toggleMovement(false); 
    
    timeElapsed = 0;
    currentSpeedMS = 0;
    maxSpeedKPH = 0;
    distanceTraveled = 0;
    currentLat = null;
    currentLon = null;
    currentAlt = null;
    previousAltitude = null; 
    verticalSpeedMS = 0; 
    currentAccuracy = 9999;
    currentAltAccuracy = 9999;
    previousTimestamp = null;

    const timeEl = document.getElementById('time-s');
    if (timeEl) timeEl.textContent = '0 s';

    const vinstEl = document.getElementById('vitesse-inst');
    if (vinstEl) vinstEl.textContent = '0.00 km/h'; 

    const v3dEl = document.getElementById('vitesse-3d');
    if (v3dEl) v3dEl.textContent = '0.00 km/h'; 

    const vmoyEl = document.getElementById('vitesse-moy');
    if (vmoyEl) vmoyEl.textContent = '0.00 km/h';

    const vmaxEl = document.getElementById('vitesse-max');
    if (vmaxEl) vmaxEl.textContent = '0.00 km/h';

    const vmsEl = document.getElementById('vitesse-ms');
    if (vmsEl) vmsEl.textContent = '0.00 m/s';

    const vmmsEl = document.getElementById('vitesse-mms');
    if (vmmsEl) vmmsEl.textContent = '0 mm/s';

    const machEl = document.getElementById('mach-number');
    if (machEl) machEl.textContent = '0.000';

    const plEl = document.getElementById('pourcent-lumiere');
    if (plEl) plEl.textContent = '0.00000000%';

    const distkmEl = document.getElementById('distance-km');
    if (distkmEl) distkmEl.textContent = '0.000 km';

    const distmEl = document.getElementById('distance-m');
    if (distmEl) distmEl.textContent = '-- m';

    const dslEl = document.getElementById('distance-sl');
    if (dslEl) dslEl.textContent = '-- s lumière';
    
    const vvertEl = document.getElementById('vitesse-vert');
    if (vvertEl) {
        vvertEl.textContent = '0.00 m/s'; 
        vvertEl.classList.remove('warning');
    }

    const gpsStatusEl = document.getElementById('gps-status');
    if (gpsStatusEl) {
        gpsStatusEl.textContent = 'En attente...';
        gpsStatusEl.classList.add('warning');
    }
}

// --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
function initializeCockpit() {
    getGeoLocation(); 
    startBubbleLevel(); 
    updateCelestialData(); 
    
    setInterval(updateAtomicTime, 10); 
    
    const startBtn = document.getElementById('start-btn');
    const stopBtn = document.getElementById('stop-btn');
    if (startBtn) startBtn.disabled = false;
    if (stopBtn) stopBtn.disabled = true;
}

window.onload = initializeCockpit;
