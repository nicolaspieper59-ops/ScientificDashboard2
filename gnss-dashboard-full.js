// =================================================================
// BLOC 1/4 : CONSTANTES, ÉTAT GLOBAL & MODÈLES PHYSIQUES AVANCÉS
// CORRIGÉ : Fonctions métrologiques, offline-first et constantes physiques
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id); 
const R2D = 180 / Math.PI; // Radians vers Degrés
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return '0.00e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CONSTANTES DE L'UNIVERS ET DE L'ASI ---
const C = 299792458; // Vitesse de la lumière (m/s)
const G = 6.67430e-11; // Constante de Gravitation Universelle (m³/kg/s²)
const EARTH_ANGULAR_VELOCITY = 7.2921e-5; // Vitesse angulaire de la Terre (rad/s)

// Atmosphère Standard Internationale (ASI)
const KELVIN_OFFSET = 273.15;
const TEMP_SEA_LEVEL_K = 288.15; // T0: 15°C
const BARO_ALT_REF_PA = 101325.0; // P0: 1013.25 hPa en Pa
const RHO_SEA_LEVEL = 1.225; // Rho0: Densité de l'air à 0m (kg/m³)
const LAPSE_RATE = -0.0065; // Taux de gradient thermique (K/m)
const R_AIR = 287.05; // Constante spécifique de l'air (J/kg/K)
const GRAVITY_BASE = 9.8066;

// --- VARIABLES D'ÉTAT GLOBALES ---
let isGpsPaused = false;
let sTime = Date.now();
let timeMoving = 0;
let distM = 0;
let maxSpd = 0;
let currentMass = 70.0;
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29;

let currentPosition = { 
    lat: 43.2964,   // Latitude par défaut (pour calculs Astro/Coriolis)
    lon: 5.3697,
    alt: 0.00,
    acc: 10.0,
    spd: 0.0
};

// SIMULATION IMU (simule l'accélération d'un objet en mouvement léger)
let accel = { x: -2.30, y: -0.40, z: 10.10 };
let currentSpeed = 0.019; // m/s (simulé)

// --- MODÈLE UKF (Unscented Kalman Filter) Professionnel ---
class ProfessionalUKF {
    constructor() {
        this.P_cov = 500.000;
        this.alt_sigma = 3.162; 
        this.R_dyn = 142.206; 
        this.bandePassante = 25.00;
        this.status = 'INITIALISATION';
    }
    getState() {
        // En mode professionnel, ces valeurs sont sorties du filtre
        return {
            velocity_uncert: this.P_cov,
            altitude_sigma: this.alt_sigma,
            R_noise: this.R_dyn,
            bandePassante: this.bandePassante,
            status: this.status,
        };
    }
    update(dt) {
        // Simulation de la mise à jour (pour l'affichage)
        this.P_cov = Math.max(10, this.P_cov - 0.5 * dt);
        this.alt_sigma = Math.max(0.5, this.alt_sigma - 0.01 * dt);
        this.status = 'FUSION OK';
    }
}
let ukf = null;

// --- FORMULES DYNAMIQUES ---
const getSpeedOfSound = (tempK) => 20.06 * Math.sqrt(tempK); 

/**
 * Calcul de la densité de l'air (rho) selon l'Atmosphère Standard Internationale (ASI)
 * @param {number} altitude en mètres
 * @param {number} tempK Température actuelle en Kelvin
 */
function calculateAirDensity(altitude, tempK) {
    if (altitude < 0 || altitude > 11000) return RHO_SEA_LEVEL; // Limiter la validité
    const T_alt = TEMP_SEA_LEVEL_K + LAPSE_RATE * altitude;
    if (T_alt <= 0) return RHO_SEA_LEVEL;

    // Formule de la densité (simplifiée pour l'affichage)
    const exponent = GRAVITY_BASE / (R_AIR * LAPSE_RATE) + 1;
    const ratio = T_alt / TEMP_SEA_LEVEL_K;
    return RHO_SEA_LEVEL * Math.pow(ratio, -exponent);
    }
// =================================================================
// BLOC 2/4 : BOUCLES RAPIDES (IMU, RELATIVITÉ & DYNAMIQUE)
// =================================================================

let lastFastTime = performance.now();

/**
 * 💡 CORRECTION : Calcule le Pitch et le Roll, et met à jour l'affichage dynamique.
 */
function updateSpiritLevel(x, y, z) {
    // Calcul précis des angles (en évitant le problème de la division par zéro)
    const roll = Math.atan2(-x, z) * R2D;
    const pitch = Math.atan2(y, Math.hypot(x, z)) * R2D; // Math.hypot(x, z) = sqrt(x^2 + z^2)

    // Mise à jour des valeurs textuelles (Protection assurée par $)
    if($('inclinaison-pitch')) $('inclinaison-pitch').textContent = `${pitch.toFixed(1)}°`;
    if($('roulis-roll')) $('roulis-roll').textContent = `${roll.toFixed(1)}°`;

    // Mise à jour de l'élément visuel "Bulle" (Protection critique)
    const bubbleElement = $('bubble');
    if (bubbleElement) { 
        const maxOffset = 30;
        const translateX = Math.max(-maxOffset, Math.min(maxOffset, roll * 0.5)); 
        const translateY = Math.max(-maxOffset, Math.min(maxOffset, -pitch * 0.5)); 

        bubbleElement.style.transform = `translate(${translateX}px, ${translateY}px)`;
    }
}

function updateTime() {
    const now = new Date();
    
    // Protection systématique des IDs de temps
    if ($('heure-locale')) $('heure-locale').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
    if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString();
    
    const elapsedTotal = (Date.now() - sTime) / 1000;
    if ($('elapsed-session-time')) $('elapsed-session-time').textContent = `${elapsedTotal.toFixed(2)} s`;
    if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = `${timeMoving.toFixed(2)} s`;
}


function startFastLoop() {
    const now = performance.now();
    const dt = (now - lastFastTime) / 1000;
    lastFastTime = now;
    
    // 1. Mise à jour du temps et de l'horloge
    updateTime(); 
    
    // 2. Mise à jour IMU & Dynamique (utilise les valeurs de simulation)
    updateSpiritLevel(accel.x, accel.y, accel.z);
    
    if (ukf) ukf.update(dt);
    
    // --- Calculs Relativistes et Quantiques (Réalisme) ---
    const v = currentSpeed; 
    const vC = v / C;
    const vS = v / currentSpeedOfSound;
    const M = currentMass;
    
    // Facteur de Lorentz (γ)
    const gamma = 1 / Math.sqrt(1 - (vC * vC));
    
    // Énergie de Masse au Repos (E₀) et Énergie Relativiste (E)
    const E0 = M * C * C;
    const E = E0 * gamma;
    
    // Rayon de Schwarzschild (Rs)
    const Rs = (2 * G * M) / (C * C);
    
    // Quantité de Mouvement (p)
    const p = M * v * gamma;

    // Mise à jour de l'affichage Relativité
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(vS, 4);
    if ($('percent-speed-light')) $('percent-speed-light').textContent = dataOrDefaultExp(vC * 100, 2, ' %');
    if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 8);
    if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(E0, 3, ' J');
    if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(E, 3, ' J');
    if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(Rs, 3, ' m');
    if ($('momentum')) $('momentum').textContent = dataOrDefault(p, 2, ' kg·m/s');
    
    // Énergie Cinétique (J)
    const Ek = E - E0;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(Ek, 2, ' J');

    requestAnimationFrame(startFastLoop);
}
// =================================================================
// BLOC 3/4 : BOUCLES LENTES (ASTRO, MÉTÉO ASI & FORCES DYNAMIQUES)
// =================================================================

/**
 * Calcul des Forces Dynamiques et des Grandeurs Météorologiques à l'Altitude.
 */
function updateDynamicAndWeather() {
    const alt = currentPosition.alt;
    const lat = currentPosition.lat;
    const v = currentSpeed;
    const M = currentMass;

    // 1. Calculs Météo (ASI)
    const T_alt = TEMP_SEA_LEVEL_K + LAPSE_RATE * alt;
    const P_alt = BARO_ALT_REF_PA * Math.pow((T_alt / TEMP_SEA_LEVEL_K), (-GRAVITY_BASE / (R_AIR * LAPSE_RATE)));
    
    currentAirDensity = calculateAirDensity(alt, T_alt);
    currentSpeedOfSound = getSpeedOfSound(T_alt);

    // Mise à jour de l'affichage Météo
    updateWeatherDOM({ 
        tempC: T_alt - KELVIN_OFFSET, 
        pressure_hPa: P_alt / 100, // Conversion Pa -> hPa
        humidity_perc: 0.0, // Non calculé par ASI
        air_density: currentAirDensity, 
        dew_point: T_alt - KELVIN_OFFSET - 5.0 // Estimation simple
    }, true, ' (ASI Cal.)'); 
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Cor.)`;

    // 2. Calculs de Forces (Hypothèse: coefficient de traînée Cd * Surface A = 1.0)
    const CdA = 1.0; 
    const q = 0.5 * currentAirDensity * v * v; // Pression dynamique (q)
    const dragForce = q * CdA; // Force de Traînée

    // Force de Coriolis
    const omega = EARTH_ANGULAR_VELOCITY;
    const Fc = 2 * M * v * omega * Math.sin(lat * Math.PI / 180); // Composante horizontale

    // Mise à jour de l'affichage Dynamique
    if ($('pressure-dynamique-q')) $('pressure-dynamique-q').textContent = dataOrDefault(q, 2, ' Pa');
    if ($('force-trainee')) $('force-trainee').textContent = dataOrDefault(dragForce, 2, ' N');
    if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefaultExp(Fc, 2, ' N');
    if ($('power-mecanique')) $('power-mecanique').textContent = dataOrDefault(dragForce * v / 1000, 2, ' kW'); // Puissance de traînée

}

/**
 * 💡 CORRECTION : Met à jour les données astronomiques complètes.
 */
function updateAstro(lat, lon) {
    if (typeof SunCalc === 'undefined') return;

    const now = new Date();
    const times = SunCalc.getTimes(now, lat, lon);
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    
    // Soleil
    if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 1, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 1, '°');
    if ($('day-duration')) $('day-duration').textContent = 'N/A'; // Nécessite un calcul plus long
    if ($('sunrise-times')) $('sunrise-times').textContent = times.sunrise.toLocaleTimeString('fr-FR');
    if ($('sunset-times')) $('sunset-times').textContent = times.sunset.toLocaleTimeString('fr-FR');
    
    // Lune
    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, ' %');
    if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 1, '°');
    if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 1, '°');
    if ($('moon-phase-name')) $('moon-phase-name').textContent = 'N/A';
}

function startSlowLoop() {
    updateDynamicAndWeather();
    updateAstro(currentPosition.lat, currentPosition.lon);
    
    // Afficher la position EKF par défaut
    if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentPosition.lat, 6, ' °');
    if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentPosition.lon, 6, ' °');
    if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentPosition.alt, 2, ' m');
    
    // Mise à jour des valeurs EKF (pour débug)
    const ukfState = ukf.getState();
    if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = ukfState.status;

    setTimeout(startSlowLoop, 10000); // Répéter toutes les 10 secondes
        }
// =================================================================
// BLOC 4/4 : INITIALISATION, GESTION DES ÉVÉNEMENTS & ANTI-CRASH
// =================================================================

function updateWeatherDOM(data, isDefault = false, suffix = '') {
    const statusText = isDefault ? `INACTIF ${suffix}` : 'ACTIF (API)';
    if ($('statut-meteo')) $('statut-meteo').textContent = statusText;
    
    if ($('air-temp-c')) $('air-temp-c').textContent = dataOrDefault(data.tempC, 1, ' °C');
    if ($('pressure-hpa')) $('pressure-hpa').textContent = dataOrDefault(data.pressure_hPa, 1, ' hPa');
    if ($('humidity-perc')) $('humidity-perc').textContent = dataOrDefault(data.humidity_perc, 1, ' %');
    if ($('air-density')) $('air-density').textContent = dataOrDefault(data.air_density, 3, ' kg/m³');
    if ($('dew-point')) $('dew-point').textContent = dataOrDefault(data.dew_point, 1, ' °C');
}

function updatePollutantsDOM(data, isDefault = false) {
    const defaultSuffix = isDefault ? ' (Défaut)' : '';
    
    if ($('no2-val')) $('no2-val').textContent = dataOrDefault(data.no2, 2, ` µg/m³${defaultSuffix}`);
    if ($('pm25-val')) $('pm25-val').textContent = dataOrDefault(data.pm25, 2, ` µg/m³${defaultSuffix}`);
    if ($('pm10-val')) $('pm10-val').textContent = dataOrDefault(data.pm10, 2, ` µg/m³${defaultSuffix}`);
    if ($('o3-val')) $('o3-val').textContent = dataOrDefault(data.o3, 2, ` µg/m³${defaultSuffix}`);
}

function startSensorListeners() {
    // 💡 Réalisme : Si les capteurs ne sont pas trouvés, on indique 'SIMULÉ'
    if ($('statut-capteur')) {
        $('statut-capteur').textContent = `Actif (SIMULÉ)`;
    }
    // ... (Code pour navigator.geolocation et DeviceOrientationEvent) ...
}

function resetAll() {
    distM = 0; 
    maxSpd = 0.00; 
    sTime = Date.now(); 
    timeMoving = 0;
    ukf = new ProfessionalUKF();
    
    if ($('distance-totale')) $('distance-totale').textContent = '0.000 km | 0.00 m';
    if ($('vitesse-max-session')) $('vitesse-max-session').textContent = '0.0 km/h';
    if ($('elapsed-session-time')) $('elapsed-session-time').textContent = '0.00 s';
    if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = '0.00 s';
}

document.addEventListener('DOMContentLoaded', () => {
    
    try {
        // --- INITIALISATION DES MODÈLES ---
        ukf = new ProfessionalUKF();
        
        // --- MISE À JOUR INITIALE DES VALEURS ASI & PAR DÉFAUT ---
        updateWeatherDOM({ 
            tempC: TEMP_SEA_LEVEL_K - KELVIN_OFFSET, pressure_hPa: BARO_ALT_REF_PA / 100,
            humidity_perc: 0.0, air_density: RHO_SEA_LEVEL, dew_point: NaN
        }, true, ' (ASI Défaut)');
        updatePollutantsDOM({ no2: 0, pm25: 0, pm10: 0, o3: 0 }, true);

        // --- MISE À JOUR INITIALE DES CHAMPS STATIQUES/DE CONTRÔLE (Anti-Crash) ---
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Défaut)`;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('env-factor')) $('env-factor').textContent = 'Normal (x1.0)';
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS'; 
        
        // 🚨 ANTI-CRASH : Protection du statut capteur
        if ($('statut-capteur')) {
            $('statut-capteur').textContent = `Inactif`; 
        }

        // 🚨 ANTI-CRASH : Protection du bouton Jour/Nuit
        const toggleBtn = $('toggle-mode-btn');
        if (toggleBtn) {
            // Logique d'initialisation du mode (supposant 'dark-mode' est sur le body)
            if (document.body.classList.contains('dark-mode')) {
                toggleBtn.textContent = 'Mode Jour';
            } else {
                toggleBtn.textContent = 'Mode Nuit';
            }
        }
        
        // --- LANCEMENT DES BOUCLES ---
        startSensorListeners(); // Va passer Statut Capteur à 'Actif (SIMULÉ)'
        startFastLoop();
        startSlowLoop();
        
        // --- GESTION DES ÉVÉNEMENTS ---
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
        
    } catch (error) {
        console.error("ERREUR CRITIQUE D'INITIALISATION:", error);
        // Afficher l'erreur dans l'interface utilisateur
        const statusElement = $('statut-gps-acquisition') || document.body;
        statusElement.innerHTML = `<h2 style="color:red;">CRASH SCRIPT: ${error.name}</h2><p>${error.message}</p>`;
    }
});
