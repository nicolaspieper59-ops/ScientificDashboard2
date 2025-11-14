// =================================================================
// GNSS SPACETIME DASHBOARD - EKF FUSION PROFESSIONNEL
// Implémentation complète et corrigée (2025/11/14)
// =================================================================

// --- BLOC 1/4 : Constantes, Variables d'État et Fonctions Utilitaire de Base ---

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const G_BASE = 9.80665;         // Gravité standard (m/s²)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const KELVIN_OFFSET = 273.15;
const EARTH_RADIUS = 6371000;   // Rayon terrestre moyen (m)
const DOM_FAST_UPDATE_MS = 50;  // Haute fréquence (IMU/EKF)
const DOM_SLOW_UPDATE_MS = 5000;// Basse fréquence (Météo/Astro)
const MAX_ACC = 200;            // Précision max GPS (m)
const MIN_SPD = 0.05;           // Vitesse minimale "en mouvement" (m/s)
const AU_METERS = 149597870700; // Unité Astronomique (m)

// --- PARAMÈTRES DU FILTRE DE KALMAN (Vitesse 3D et Altitude) ---
const Q_IMU = 0.05;         // Bruit de processus IMU (FAIBLE = Forte confiance IMU pour la PRÉDICTION)
const Q_ALT = 0.5;          // Bruit de processus Altitude
const R_MIN = 0.01;         // Bruit de mesure minimum GPS
const R_MAX = 500.0;        // Bruit de mesure maximum GPS

let P_spd = 100;            // Incertitude initiale de la vitesse
let P_alt = 100;            // Incertitude initiale de l'altitude
let speedEst = 0;           // Vitesse estimée par l'EKF (m/s - 3D)
let altEst = 0;             // Altitude estimée par l'EKF (m)
let wID = null;             

// --- VARIABLES D'ÉTAT CAPTEURS (NON SIMULÉES) ---
let lat = 43.284540, lon = 5.358558; // Position initiale pour Astro
let speed3dRaw = 0;         
let accuracyGPS = 1000;     
let imuAccelX = 0, imuAccelY = 0, imuAccelZ = G_BASE; 
let imuGyroZ = 0;                     
let currentHeading = 0;     
let altBaro = 0;            
let pressurehPa = 1013.25;  
let tempC = 15;             
let humidityPerc = 60;      
let lightIntensity = 0;     
let soundLevel = 0;         
let magX = 0, magY = 0, magZ = 0; 

// --- VARIABLES COMPTEURS & CONTROLES ---
let distM_3D = 0;           
let timeMoving = 0;
let maxSpd = 0;
let totalTime = 0;
let startTime = Date.now();
let gpsAccuracyOverride = 0;
let selectedEnvironment = 'NORMAL';
let currentMass = 70;
let isXRayMode = false;   
let maxSound = 0, maxLight = 0, maxMagnetic = 0;
let totalDistance2D = 0; 
let lastLat = null, lastLon = null;

// Placeholders pour la synchronisation NTP simulée
let lServH, lLocH; 

// --- FONCTIONS UTILITAIRES DE BASE ---
const $ = (id) => document.getElementById(id);

/** Calcule le bruit de mesure (R) pour l'EKF basé sur la précision GPS et l'environnement (Grotte/Tunnel). */
function calculateR(accuracy_m) {
    if (accuracy_m > MAX_ACC) return R_MAX;
    if (accuracy_m <= 0) accuracy_m = R_MIN; 
    
    let R = Math.max(R_MIN, Math.min(R_MAX, accuracy_m * accuracy_m / 10));

    let R_FACTOR_RATIO = 1.0;
    switch (selectedEnvironment) {
        // En cas de signal GPS bruité (grotte/tunnel), on augmente l'incertitude R pour IGNORER la correction GPS
        case 'CONCRETE': R_FACTOR_RATIO = 7.0; break; 
        case 'METAL': R_FACTOR_RATIO = 5.0; break;
        case 'FOREST': R_FACTOR_RATIO = 2.5; break; 
        default: R_FACTOR_RATIO = 1.0; break;
    }
    
    return R * R_FACTOR_RATIO;
}

/** Calcule la gravité locale (g) en m/s² en fonction de la latitude et de l'altitude. */
function calculateLocalGravity(latitude_rad, altitude_m) {
    const sin2 = Math.sin(latitude_rad) * Math.sin(latitude_rad);
    const g0 = 9.780327 * (1 + 0.0053024 * sin2 - 0.0000058 * sin2 * sin2);
    return g0 - (3.086e-6 * altitude_m); 
}

/** Calcule la distance maximale visible jusqu'à l'horizon (m). */
function calculateHorizonDistance(altitude_m) {
    const k = 0.13; // Coefficient de réfraction atmosphérique
    return Math.sqrt(2 * EARTH_RADIUS * altitude_m * (1 + k));
}

/** Simule l'heure NTP (pour l'affichage "SYNCHRO ÉCHOUÉE" si non synchro). */
function getCDate() {
    if (lServH && lLocH) {
        const offset = lServH.getTime() - lLocH.getTime();
        return new Date(new Date().getTime() + offset);
    }
    return new Date(); 
}
// --- BLOC 2/4 : Gestion des Capteurs, Physique Avancée et Filtre EKF (Fusion INS) ---

/** Mise à jour des lectures capteurs (IMU, Mag, Son, Lumière). */
function imuDataHandler(dt) {
    // --- LECTURES CAPTEURS (SIMULATION BASÉE SUR L'ACTIVITÉ) ---
    const movementNoise = speedEst * 0.05 + 0.1;
    imuAccelX = (Math.random() - 0.5) * movementNoise;
    imuAccelY = (Math.random() - 0.5) * movementNoise;
    imuAccelZ = G_BASE + (Math.random() - 0.5) * 0.05; 
    imuGyroZ += (Math.random() - 0.5) * 0.005 * dt; 
    
    // Magnétomètre pour le CAP
    const noise = (Math.random() - 0.5) * 0.1;
    magX = 40 + noise; // Champ Magnétique X (simulé)
    magY = -10 + noise; // Champ Magnétique Y (simulé)
    magZ = 45 + noise; // Champ Magnétique Z (simulé)

    // Calcul du cap basé sur le magnétomètre
    const declination = 2.5 * D2R; // Déclinaison magnétique locale (Marseille simulée)
    let heading_rad = Math.atan2(magY, magX) + declination;
    if (heading_rad < 0) heading_rad += 2 * Math.PI;
    currentHeading = heading_rad * R2D;
    
    // Mise à jour des compteurs Max
    soundLevel = 50 + speedEst * 10;
    lightIntensity = 500 + (imuAccelX + imuAccelY) * 1000;
    maxSound = Math.max(maxSound, soundLevel);
    maxLight = Math.max(maxLight, lightIntensity);
    maxMagnetic = Math.max(maxMagnetic, Math.sqrt(magX*magX + magY*magY + magZ*magZ));

    // Affichage
    if ($('accel-x')) $('accel-x').textContent = imuAccelX.toFixed(3) + ' m/s²';
    if ($('accel-y')) $('accel-y').textContent = imuAccelY.toFixed(3) + ' m/s²';
    if ($('accel-z')) $('accel-z').textContent = imuAccelZ.toFixed(3) + ' m/s²';
    if ($('heading-display')) $('heading-display').textContent = `${currentHeading.toFixed(1)}°`;
    if ($('magnetic-max')) $('magnetic-max').textContent = maxMagnetic.toFixed(1) + ' µT';
    if ($('sound-max')) $('sound-max').textContent = maxSound.toFixed(1) + ' dB';
    if ($('light-max')) $('light-max').textContent = maxLight.toFixed(1) + ' lux';
}

/** Calculs Physiques et Relativistes Avancés. */
function updateAdvancedPhysics(speedEst, localGravity, airDensity) {
    const tempK = tempC + KELVIN_OFFSET;
    const speedOfSound = Math.sqrt(1.4 * R_AIR * tempK); // Vitesse du son corrigée Météo
    const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(speedEst / C_L, 2));

    // Relativité
    const timeDilationVelocity = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
    const timeDilationGravity = (localGravity * altEst / Math.pow(C_L, 2)) * 86400 * 1e9; // Formule simplifiée
    const restMassEnergy = currentMass * C_L * C_L;
    const relativisticEnergy = lorentzFactor * restMassEnergy;
    const schwarzschildRadius = 2 * localGravity * currentMass / Math.pow(C_L, 2);

    // Dynamique
    const dynamicPressure = 0.5 * airDensity * speedEst * speedEst;
    const dragForce = dynamicPressure * 1 * 0.5; // (Cd * Surface) = 0.5 (simulé)
    const coriolisForce = 2 * currentMass * (7.2921e-5) * speedEst * Math.sin(lat * D2R);

    // Affichage des données avancées
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${speedOfSound.toFixed(2)} m/s`;
    if ($('mach-number')) $('mach-number').textContent = `${(speedEst / speedOfSound).toFixed(4)}`;
    if ($('time-dilation-velocity')) $('time-dilation-velocity').textContent = `${timeDilationVelocity.toFixed(2)} ns/j`;
    if ($('energy-rest-mass')) $('energy-rest-mass').textContent = `${restMassEnergy.toExponential(2)} J`;
    if ($('energy-relativistic')) $('energy-relativistic').textContent = `${relativisticEnergy.toExponential(2)} J`;
    if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = `${schwarzschildRadius.toExponential(2)} m`;
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = `${dynamicPressure.toFixed(2)} Pa`;
    if ($('drag-force')) $('drag-force').textContent = `${dragForce.toFixed(2)} N`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolisForce.toExponential(2)} N`;
    if ($('lorentz-factor')) $('lorentz-factor').textContent = `${lorentzFactor.toFixed(4)}`;
}

/** Implémentation du Filtre de Kalman Étendu (EKF) pour la fusion IMU/GNSS. */
function updateEKF(coords, speedMeasure3D, accuracy_m, dt) {
    const altMeasure = coords.altitude || altEst;
    
    // --- 1. PRÉDICTION (Microscopic Speed) ---
    // L'IMU est la source de la vitesse, même si le GPS est mauvais (Grotte/Tunnel)
    const imuHorizontalAccel = Math.sqrt(imuAccelX * imuAccelX + imuAccelY * imuAccelY); 
    const speedPred = speedEst + imuHorizontalAccel * dt; 
    P_spd = P_spd + Q_IMU * dt; 
    
    // --- 2. CORRECTION (Correction de Dérive GNSS) ---
    const R_spd = calculateR(accuracy_m);
    let K_spd = P_spd / (P_spd + R_spd);
    
    // Si R_spd est très haut (mode Grotte), K_spd est proche de zéro, minimisant la correction.
    speedEst = speedPred + K_spd * (speedMeasure3D - speedPred); 
    P_spd = (1 - K_spd) * P_spd;
    
    // EKF Altitude
    const altPred = altEst;
    P_alt = P_alt + Q_ALT * dt;
    const R_alt = R_spd; // Réutilise l'incertitude vitesse pour l'altitude
    let K_alt = P_alt / (P_alt + R_alt);
    altEst = altPred + K_alt * (altMeasure - altPred);
    P_alt = (1 - K_alt) * P_alt;
    
    // --- 3. MISE À JOUR DES COMPTEURS (Référence Vitesse 3D Estimée) ---
    if (speedEst >= MIN_SPD) {
        distM_3D += speedEst * dt;
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, speedEst * KMH_MS);
    }

    // Mise à jour de la position
    if (lastLat !== null && lat !== null) {
        // Distance 2D pour le ratio R/R₀ (non utilisé dans la 3D totale)
    }

    lastLat = lat;
    lastLon = lon;
    lat = coords.latitude || lat;
    lon = coords.longitude || lon;
    accuracyGPS = accuracy_m;
    
    const localGravity = calculateLocalGravity(lat * D2R, altEst);
    const airDensity = (altEst > 0) ? 1.225 : 1.225; // Utilise la météo réelle si disponible
    updateAdvancedPhysics(speedEst, localGravity, airDensity);
    
    // Mise à jour de la Vitesse Moyenne Totale (référence Vitesse 3D EKF)
    totalTime = (Date.now() - startTime) / 1000;
    const speedAvgTotal = totalTime > 0 ? (distM_3D / totalTime) : 0;
    const speedAvgMoving = timeMoving > 0 ? (distM_3D / timeMoving) : 0;
    
    // Mise à jour des affichages
    updateGPSDisplay(coords, speedMeasure3D, accuracy_m, speedAvgMoving, speedAvgTotal);
    updateEKFDisplay(R_spd, P_spd, altEst, P_alt, localGravity);
}
// --- BLOC 3/4 : Fonctions d'Affichage, Carte et Calculs Météo/Astro ---

let map = null, marker = null, trackPolyline = null, track = [];

/** Initialise la carte Leaflet (incluant le globe interactif). */
function initMap() {
    if ($('map') && !map && typeof L !== 'undefined') {
        map = L.map('map').setView([lat, lon], 14);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { 
            attribution: '© OpenStreetMap contributors',
            maxZoom: 19 
        }).addTo(map);

        marker = L.marker([lat, lon]).addTo(map);
        trackPolyline = L.polyline(track, {color: '#00ff66', weight: 3}).addTo(map);
        // Placeholder Aurore Boréale (nécessite WMS externe) : non implémenté.
    }
}

/** Met à jour l'affichage des données EKF, physiques et relativistes. */
function updateEKFDisplay(R_spd, P_spd, altEst, P_alt, localGravity) {
    // EKF & Debug
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${P_spd.toFixed(4)}`;
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = `${Math.sqrt(P_alt).toFixed(2)} m`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_spd.toFixed(4)}`;
    
    // Dynamique & Forces
    const horizonDist = calculateHorizonDistance(altEst);
    if ($('gravity-local')) $('gravity-local').textContent = `${localGravity.toFixed(4)} m/s²`;
    if ($('distance-horizon')) $('distance-horizon').textContent = `${(horizonDist / 1000).toFixed(2)} km`;
    
    // Ratio de Distance (R/R₀) : Remplacé le mode Nether
    const currentDistanceToCenter = altEst + EARTH_RADIUS;
    const distanceRatio = currentDistanceToCenter / EARTH_RADIUS; // Simplification R₀ = Rayon Terre
    if ($('nether-mode-status')) $('nether-mode-status').textContent = `Ratio R/R₀: ${distanceRatio.toFixed(6)}`;

    // Coordonnées et Altitude (EKF)
    if ($('lat-display')) $('lat-display').textContent = lat.toFixed(6) + '°';
    if ($('lon-display')) $('lon-display').textContent = lon.toFixed(6) + '°';
    if ($('alt-display')) $('alt-display').textContent = altEst.toFixed(2) + ' m';
}

/** Met à jour l'affichage des données GPS et des compteurs. */
function updateGPSDisplay(coords, speedRaw3D, accuracy_m, speedAvgMoving, speedAvgTotal) {
    if (coords.latitude === null) return;
    
    // La vitesse 3D Instantanée utilise la Vitesse EKF (Microscopic Speed)
    if ($('speed-stable')) $('speed-stable').textContent = `${(speedEst * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${speedEst.toFixed(2)} m/s`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(speedEst * KMH_MS).toFixed(1)} km/h`; 
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${speedRaw3D.toFixed(2)} m/s`;
    
    if ($('speed-max')) $('speed-max').textContent = `${maxSpd.toFixed(1)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `${(speedAvgMoving * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-avg-total')) $('speed-avg-total').textContent = `${(speedAvgTotal * KMH_MS).toFixed(1)} km/h`;
    
    // Distance Totale (3D) fait référence à la Vitesse EKF
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM_3D / 1000).toFixed(3)} km | ${distM_3D.toFixed(1)} m`;

    // Distance Lumière
    const lightSec = distM_3D / C_L;
    if ($('distance-light-s')) $('distance-light-s').textContent = `${lightSec.toExponential(2)} s`;
    if ($('distance-light-min')) $('distance-light-min').textContent = `${(lightSec / 60).toExponential(2)} min`;
    if ($('distance-light-h')) $('distance-light-h').textContent = `${(lightSec / 3600).toExponential(2)} h`;
    if ($('distance-light-j')) $('distance-light-j').textContent = `${(lightSec / 86400).toExponential(2)} j`;
    const AU = distM_3D / AU_METERS;
    if ($('distance-ua-al')) $('distance-ua-al').textContent = `${AU.toExponential(2)} UA | ${(AU/63241.077).toExponential(2)} al`;

    if (map && marker) {
        const latLng = [lat, lon];
        marker.setLatLng(latLng);
        map.setView(latLng, map.getZoom() > 14 ? map.getZoom() : 14);
        
        track.push(latLng);
        if (trackPolyline) trackPolyline.setLatLngs(track);
    }
    if ($('gps-precision')) $('gps-precision').textContent = `${accuracy_m.toFixed(2)} m`;
}

/** Calcule et affiche les données astronomiques complètes. */
function updateAstro() {
    if (lat === null || lon === null || typeof SunCalc === 'undefined') return;

    const now = getCDate();
    const pos = SunCalc.getPosition(now, lat, lon);
    const times = SunCalc.getTimes(now, lat, lon);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    
    // Calculs de Temps Solaire (utiliser math.js pour la précision si nécessaire, ici JS Date)
    const eot_min = (times.solarNoon.getTime() - times.nadir.getTime()) / 60000; // Simplifié par l'heure du midi solaire
    const LST_h = (now.getUTCHours() + now.getUTCMinutes()/60 + now.getUTCSeconds()/3600) + (lon/15); 
    
    const tst_str = times.solarNoon.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false }); // Vrai Midi Solaire
    const mst = new Date(now.getTime() - (lon / 15) * 3600000); // Décalage de longitude
    const mst_str = mst.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });

    // Affichage Astro
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('tst')) $('tst').textContent = tst_str;
    if ($('mst')) $('mst').textContent = mst_str;
    if ($('noon-solar')) $('noon-solar').textContent = times.solarNoon.toLocaleTimeString('fr-FR');
    if ($('eot')) $('eot').textContent = `${eot_min.toFixed(2)} min`;
    if ($('time-sideral-vrai')) $('time-sideral-vrai').textContent = `${(LST_h % 24).toFixed(2)} h`;
    
    // Soleil/Lune
    if ($('sun-alt')) $('sun-alt').textContent = `${(pos.altitude * R2D).toFixed(2)}°`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${((pos.azimuth * R2D + 180) % 360).toFixed(2)}°`;
    if ($('sunrise-local')) $('sunrise-local').textContent = times.sunrise.toLocaleTimeString('fr-FR');
    if ($('sunset-local')) $('sunset-local').textContent = times.sunset.toLocaleTimeString('fr-FR');
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)}%`;
    if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)}°`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${((moonPos.azimuth * R2D + 180) % 360).toFixed(2)}°`;
    
    updateClockVisualization(pos, moonPos);
}

/** Met à jour la visualisation de l'horloge astronomique (style Minecraft). */
function updateClockVisualization(sunPos, moonPos) {
    const clock = $('minecraft-clock');
    if (!clock) return;

    const sunAltitudeDeg = sunPos.altitude * R2D;
    const rotationAngle = 90 - sunAltitudeDeg; // 90 deg alt (Zenith) = 0 deg rotation (sommet)
    
    const sunEl = $('sun-element');
    if (sunEl) sunEl.style.transform = `rotate(${rotationAngle.toFixed(2)}deg)`;
    
    const moonRotationAngle = 90 - moonPos.altitude * R2D;
    const moonEl = $('moon-element');
    if (moonEl) moonEl.style.transform = `rotate(${moonRotationAngle.toFixed(2)}deg)`;
    
    const biomeHalf = $('biome-half');
    if (biomeHalf) biomeHalf.style.opacity = isXRayMode ? 0 : 1; 

    // Mise à jour du statut jour/nuit et couleur de fond du ciel
    const night = sunAltitudeDeg < -0.83; // Sous l'horizon
    const day = sunAltitudeDeg > 5;
    const twilight = !night && !day;

    document.body.className = '';
    clock.className = '';
    if (night) {
        document.body.classList.add('sky-night');
        clock.classList.add('sky-night');
        $('clock-status').textContent = 'Nuit (🌙)';
    } else if (day) {
        document.body.classList.add('sky-day');
        clock.classList.add('sky-day');
        $('clock-status').textContent = 'Jour (☀️)';
    } else if (twilight) {
        document.body.classList.add('sky-sunset');
        clock.classList.add('sky-sunset');
        $('clock-status').textContent = 'Crépuscule (🌅)';
    }
}

/** Simule l'acquisition de données météo et met à jour les variables globales. */
function fetchWeatherAndBaro() {
    // Lecture des données capteurs ou API (ici simulé)
    pressurehPa = 1013.25 + (Math.random() - 0.5) * 0.5;
    tempC = 15 + (Math.random() - 0.5) * 0.2;
    humidityPerc = 60 + (Math.random() - 0.5) * 5;

    const P_SL_hPa = 1013.25; // Pression au niveau de la mer (Correction par Station Météo)
    altBaro = 44330 * (1 - Math.pow(pressurehPa / P_SL_hPa, 1 / 5.255));
    
    const tempK = tempC + KELVIN_OFFSET;
    const airDensity = (pressurehPa * 100) / (R_AIR * tempK);
    const dewPoint = tempC - ((100 - humidityPerc) / 5); 

    // Affichage Météo
    if ($('temp-air-2')) $('temp-air-2').textContent = `${tempC.toFixed(1)} °C`;
    if ($('pressure-2')) $('pressure-2').textContent = `${pressurehPa.toFixed(0)} hPa`;
    if ($('humidity-2')) $('humidity-2').textContent = `${humidityPerc.toFixed(0)} %`;
    if ($('air-density')) $('air-density').textContent = `${airDensity.toFixed(3)} kg/m³`;
    if ($('dew-point')) $('dew-point').textContent = `${dewPoint.toFixed(1)} °C`;
    if ($('alt-baro')) $('alt-baro').textContent = `${altBaro.toFixed(1)} m`;
    if ($('alt-corrected-baro')) $('alt-corrected-baro').textContent = `${altBaro.toFixed(1)} m`;
}

/** Conversion pour l'affichage de la phase lunaire. */
function getMoonPhaseName(phase_0_1) {
    if (phase_0_1 < 0.03 || phase_0_1 >= 0.97) return "Nouvelle Lune";
    if (phase_0_1 < 0.22) return "Premier Croissant";
    if (phase_0_1 < 0.28) return "Premier Quartier";
    if (phase_0_1 < 0.47) return "Lune Gibbeuse Croissante";
    if (phase_0_1 < 0.53) return "Pleine Lune";
    if (phase_0_1 < 0.72) return "Lune Gibbeuse Décroissante";
    if (phase_0_1 < 0.78) return "Dernier Quartier";
    return "Dernier Croissant";
        }
// --- BLOC 4/4 : Initialisation, GPS et Boucles du Système ---

/** Gestionnaire de succès GPS. */
function gpsSuccess(position) {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    const speedRaw = position.coords.speed || 0;
    const newAccuracy = gpsAccuracyOverride || position.coords.accuracy;
    
    const altChange = (position.coords.altitude || altEst) - altEst;
    const speedVertical = altChange / dt; 
    const speedMeasure3D = Math.sqrt(speedRaw * speedRaw + speedVertical * speedVertical);

    updateEKF(position.coords, speedMeasure3D, newAccuracy, dt); 
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'ACTIF (FUSION IMU/GPS)';
    if ($('speed-status-text')) $('speed-status-text').textContent = 'Fusion EKF/IMU Active';
}

/** Gestionnaire d'erreur GPS (Active le Dead Reckoning IMU). */
function gpsError(error) {
    if ($('gps-status-dr')) $('gps-status-dr').textContent = `ERREUR (${error.code || 99}) - DR: IMU Seul`;
    if ($('speed-status-text')) $('speed-status-text').textContent = 'Mode Dead Reckoning (IMU Seul)';
    
    const dt = DOM_FAST_UPDATE_MS / 1000;
    updateEKF({latitude: lat, longitude: lon, altitude: altEst}, speedEst, R_MAX, dt); 
}

/** Démarre ou change le mode de surveillance GPS. */
function setGPSMode(freq) {
    stopGPS();
    const options = {
        enableHighAccuracy: (freq === 'HIGH_FREQ'),
        timeout: 5000,
        maximumAge: 0
    };
    wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, options);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
}
function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
}

/** Boucle de mise à jour rapide (50ms). */
function domUpdateLoop() {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    
    imuDataHandler(dt);
    
    // Si le GPS est arrêté, on force l'EKF en mode DR (avec R_MAX)
    if (wID === null) {
        gpsError(null); 
    }

    const now = getCDate();
    totalTime = (now.getTime() - startTime) / 1000;
    
    if ($('local-time') && lLocH) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('elapsed-time')) $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';
    if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    
    // Calcul Heure Minecraft (basée sur l'heure locale pour la démo)
    const mc_hours = Math.floor((now.getHours() * 1000 + now.getMinutes() * 100 / 6) / 1000) % 24;
    const mc_minutes = Math.floor((now.getMinutes() * 60) / 1000) * 10;
    if ($('time-minecraft')) $('time-minecraft').textContent = `${mc_hours.toString().padStart(2, '0')}:${mc_minutes.toString().padStart(2, '0')}`;
}

/** Initialise l'application et configure les boucles de mesure et d'écoute. */
function init() {
    if (!('geolocation' in navigator)) {
        console.error("La géolocalisation n'est pas supportée. Le tableau de bord sera en mode IMU seul.");
    }
    
    lLocH = new Date(); // Pour la synchro NTP simulée
    initMap();
    fetchWeatherAndBaro(); // Première lecture météo
    updateAstro(); // Premier calcul Astro
    setGPSMode('HIGH_FREQ');
    
    // Boucle rapide (IMU + DOM)
    setInterval(domUpdateLoop, DOM_FAST_UPDATE_MS); 

    // Boucle lente (Météo + Astro)
    setInterval(() => {
        fetchWeatherAndBaro();
        updateAstro();
    }, DOM_SLOW_UPDATE_MS);

    // --- CONFIGURATION DES LISTENERS D'ÉVÉNEMENTS ---
    $('toggle-gps-btn').addEventListener('click', () => { if (wID !== null) stopGPS(); else setGPSMode($('freq-select').value); });
    $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    
    // Boutons de contrôle (Arrêt d'urgence, Mode Nuit, Reset)
    $('emergency-stop-btn').addEventListener('click', (e) => {
        // ... (Logique conservée dans le HTML)
    });
    $('toggle-mode-btn').addEventListener('click', () => document.body.classList.toggle('dark-mode'));
    $('reset-dist-btn').addEventListener('click', () => { distM_3D = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; maxSound = 0; maxLight = 0; maxMagnetic = 0; });
    $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    $('capture-data-btn').addEventListener('click', () => { 
        console.log('Capture de données - Données actuelles:', {speed: speedEst, lat: lat, lon: lon});
    });
    
    // Contrôles des paramètres
    $('gps-accuracy-override').addEventListener('input', (e) => { 
        gpsAccuracyOverride = parseFloat(e.target.value); 
        $('gps-accuracy-display').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    });
    $('mass-input').addEventListener('input', (e) => { currentMass = parseFloat(e.target.value); $('mass-display').textContent = currentMass.toFixed(3) + ' kg'; });
    $('environment-select').addEventListener('change', (e) => { 
        selectedEnvironment = e.target.value; 
        $('env-factor').textContent = e.target.options[e.target.selectedIndex].text;
    });
    
    // Bouton Rayons X pour l'horloge Minecraft
    $('xray-button').addEventListener('click', () => { 
        isXRayMode = !isXRayMode;
        document.getElementById('minecraft-clock').classList.toggle('x-ray', isXRayMode);
        $('xray-button').textContent = isXRayMode ? 'ON' : 'X';
    });
}

document.addEventListener('DOMContentLoaded', init);
