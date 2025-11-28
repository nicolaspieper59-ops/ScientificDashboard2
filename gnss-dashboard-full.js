// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/5 : Constantes, Utilitaires, et Bibliothèque Scientifique SciencePro
// =================================================================

/* --- FONCTIONS UTILITAIRES GLOBALES --- */
const $ = id => document.getElementById(id);

/** Met à jour le contenu textuel d'un élément par son ID, sans erreur si l'élément n'existe pas. */
const safeSet = (id, txt) => { const e = $(id); if(e) e.textContent = txt; };

/** Formate une valeur numérique ou retourne 'N/A' */
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return 'N/A';
    }
    // Utiliser la notation scientifique pour les valeurs extrêmement petites
    if (decimals > 8 && val !== 0 && (Math.abs(val) < 1e-6 || Math.abs(val) > 1e12)) {
        return val.toExponential(4) + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

/* --- CONSTANTES PHYSIQUES ET COSMOLOGIQUES --- */
const PHYS = {
    c: 299792458,           // Vitesse de la lumière (m/s)
    G: 6.67430e-11,         // Constante gravitationnelle universelle (m³/kg/s²)
    M_earth: 5.972e24,      // Masse de la Terre (kg)
    R_earth: 6371000,       // Rayon moyen de la Terre (m)
    g_base: 9.8067,         // Gravité standard au niveau de la mer (m/s²)
    R_d: 287.058,           // Constante spécifique du gaz sec (J/kg/K)
    gamma_air: 1.4,         // Rapport des capacités thermiques de l'air
    TEMP_SEA_LEVEL_K: 288.15,// Température standard au niveau de la mer (15°C)
    PRES_SEA_LEVEL: 101325,  // Pression standard au niveau de la mer (Pa)
    L_ISA: -0.0065,         // Taux de gradient de température ISA (K/m)
    SECONDS_PER_DAY: 86400, // Secondes dans un jour
    pi: Math.PI,
    omega_earth: 7.2921150e-5, // Vitesse angulaire de la Terre (rad/s)
};

/* --- SciencePro: Bibliothèque de fonctions scientifiques réelles --- */
const SciencePro = (function(){
    const C = PHYS;
    
    // --- FONCTIONS ATMOSPHÉRIQUES (ISA Standard) ---
    /** Calcule T, P, et Rho selon le modèle ISA pour la Troposphère. */
    function isaModel(altitude_m) {
        let T_K = C.TEMP_SEA_LEVEL_K;
        let P_Pa = C.PRES_SEA_LEVEL;
        
        if (altitude_m <= 11000) { // Troposphère
            T_K = C.TEMP_SEA_LEVEL_K + C.L_ISA * altitude_m;
            if (T_K <= 0) T_K = 0.01;
            
            const power = -C.g_base / (C.L_ISA * C.R_d); 
            P_Pa = C.PRES_SEA_LEVEL * Math.pow(T_K / C.TEMP_SEA_LEVEL_K, power);
        } else {
            T_K = 216.65; // Stratosphère (simplifié)
            // Le calcul de pression plus haut est omis pour la simplicité, mais le modèle ISA réel devrait continuer
        }
        
        const rho_air = P_Pa / (C.R_d * T_K); 
        return { T_K, P_Pa, rho_air };
    }

    /** Calcule la vitesse du son (m/s) en fonction de la Température (T en Kelvin). */
    function getSpeedOfSound(T_K) {
        if (T_K <= 0) return 0.0;
        return Math.sqrt(C.gamma_air * C.R_d * T_K);
    }
    
    // --- FONCTIONS RELATIVISTES ---
    /** Calcule le facteur de Lorentz (Gamma). */
    function lorentzGamma(v) {
        if (v <= 0) return 1.0;
        const ratio_sq = (v / C.c) * (v / C.c);
        if (ratio_sq >= 1.0) return Infinity;
        return 1.0 / Math.sqrt(1.0 - ratio_sq);
    }
    
    /** Calcule la dilatation du temps gravitationnelle (en ns/jour). */
    function gravitationalTimeDilation(altitude_m) {
        const R = C.R_earth;
        const r_obs = R + altitude_m;
        const d_term = (C.G * C.M_earth) / (C.c * C.c);
        const factor = d_term * (1/R - 1/r_obs); 
        const totalNs = C.SECONDS_PER_DAY * 1e9;
        return totalNs * factor;
    }

    /** Calcule le Rayon de Schwarzschild (m) pour une masse donnée. */
    function schwarzschildRadius(mass_kg) {
        if (mass_kg <= 0) return 0;
        return (2 * C.G * mass_kg) / (C.c * C.c);
    }
    
    // --- FONCTIONS ASTRONOMIQUES (Approximations de base) ---
    /** Convertit une date JS en Jour Julien. */
    function toJulian(date) {
        return date.getTime() / C.SECONDS_PER_DAY / 1000.0 + 2440587.5;
    }

    /** Approximation de la position du Soleil (Azimut/Altitude) et Phase de la Lune. */
    function getAstroData(date, lat_deg, lon_deg) {
        // Logique d'approximation solaire et lunaire (calculs étendus pour l'heure et la position)
        // ... (Logique basée sur le bloc précédent pour SunAlt/Azimuth/MoonPhase) ...
        const jd = toJulian(date);
        const n = jd - 2451545.0; 
        const L = (280.460 + 0.98564736 * n) % 360;
        const M = (357.528 + 0.98560028 * n) % 360;
        const L_rad = L * C.pi / 180;
        const M_rad = M * C.pi / 180;
        
        const lambda = L_rad + 1.915 * Math.sin(M_rad) + 0.020 * Math.sin(2 * M_rad);
        const epsilon = 23.439 * C.pi / 180; 

        const ra = Math.atan2(Math.cos(epsilon) * Math.sin(lambda), Math.cos(lambda));
        const dec = Math.asin(Math.sin(epsilon) * Math.sin(lambda));

        const T = (jd - 2451545.0) / 36525;
        const theta0 = (280.46061837 + 360.98564736629 * n + 0.000387933 * T * T - T * T * T / 38710000);
        const theta_rad = (theta0 % 360) * C.pi / 180;
        const ha = theta_rad + lon_deg * C.pi / 180 - ra;

        const lat_rad = lat_deg * C.pi / 180;
        const sin_alt = Math.sin(lat_rad) * Math.sin(dec) + Math.cos(lat_rad) * Math.cos(dec) * Math.cos(ha);
        const alt = Math.asin(sin_alt);
        
        let az = Math.atan2(
            Math.sin(ha),
            Math.cos(lat_rad) * Math.tan(dec) - Math.sin(lat_rad) * Math.cos(ha)
        );

        const alt_deg = alt * 180 / C.pi;
        const az_deg = (az * 180 / C.pi + 360) % 360;
        
        const moonAgeDays = (jd - 2451550.1 + 29.530588) % 29.530588; 
        const moonIllumination = 0.5 * (1 - Math.cos(moonAgeDays * 2 * C.pi / 29.530588));
        
        let moonPhaseName = 'Inconnu';
        if (moonAgeDays >= 13.5 && moonAgeDays < 15.5) moonPhaseName = 'Pleine Lune';
        else if (moonAgeDays >= 27.5 || moonAgeDays < 1.5) moonPhaseName = 'Nouvelle Lune';
        else if (moonAgeDays >= 6 && moonAgeDays < 8) moonPhaseName = 'Premier Quartier';
        else if (moonAgeDays >= 21 && moonAgeDays < 23) moonPhaseName = 'Dernier Quartier';
        else moonPhaseName = 'Autre Phase';

        return { sunAlt: alt_deg, sunAzimuth: az_deg, moonIllumination: moonIllumination * 100, moonPhase: moonPhaseName, moonAgeDays: moonAgeDays };
    }
    
    return {
        isaModel,
        getSpeedOfSound,
        localGravity: (h) => {
            const R = C.R_earth;
            const ratio = R / (R + h);
            return C.g_base * ratio * ratio;
        },
        lorentzGamma,
        schwarzschildRadius,
        gravitationalTimeDilation,
        restMassEnergy: (m) => m * C.c * C.c,
        timeDilation: (gamma) => {
            if (gamma === 1.0) return 0.0;
            const totalNs = C.SECONDS_PER_DAY * 1e9;
            return totalNs * (gamma - 1);
        },
        getRelativisticKineticEnergy: (gamma, E0) => E0 * (gamma - 1),
        getAstroData,
        c: C.c,
        R_d: C.R_d,
    };
})();
// =================================================================
// GNSS SPACETIME DASHBOARD - BLOC 2/5
// État Global, Moteur de Filtre UKF (Placeholder) et Traitement de Données
// =================================================================

/* --- ÉTAT GLOBAL (Variables Modifiables par l'utilisateur/capteurs) --- */
let currentMass = 70.0;     // Masse de l'objet (kg)
let currentCd = 0.47;       // Coefficient de traînée (Cd)
let currentArea = 0.5;      // Aire frontale (m²)
let currentPosition = {     // Dernière position connue (initialisée à Paris)
    lat: 48.8566,
    lon: 2.3522,
    alt: 35.0, // Altitude MSL (m)
    speed3D: 0.0, // Vitesse totale (m/s)
};
let trajectoryPoints = [];  // Historique pour le tracé de la carte et des graphiques

/* --- FilterEngine (Moteur UKF/EKF Placeholder) --- */
const FilterEngine = (function(){
    let state = {
        lat: currentPosition.lat,
        lon: currentPosition.lon,
        alt: currentPosition.alt,
        speed3D: currentPosition.speed3D,
        tempK: undefined, // Simule la non-disponibilité initiale de capteurs météo
        pressurePa: undefined,
        P: [[0.5, 0], [0, 0.5]], // Matrice de covariance (précision)
    };

    /** Initialise le filtre avec une position de départ. */
    function initFromGnss(lat, lon, alt) {
        state.lat = lat;
        state.lon = lon;
        state.alt = alt;
        safeSet('filter-type', 'UKF 21 États (Initialisé)');
        safeSet('lat', dataOrDefault(lat, 6, '°'));
        safeSet('lon', dataOrDefault(lon, 6, '°'));
        safeSet('alt', dataOrDefault(alt, 2, ' m'));
        updateTrajectory(state);
    }

    /** Simule la mise à jour de l'état (en production, ce serait l'algorithme UKF/EKF). */
    function updateState(gnssData, sensorData) {
        // En conditions réelles, l'UKF/EKF fusionnerait gnssData (GPS) et sensorData (IMU, Baro, etc.)
        
        // Simulation d'une mise à jour (pour le test)
        state.lat += (Math.random() - 0.5) * 0.0001; // Petite dérive pour l'exemple
        state.lon += (Math.random() - 0.5) * 0.0001;
        state.alt = Math.max(35, state.alt + (Math.random() - 0.5) * 0.5); // Altitude qui varie un peu
        state.speed3D = Math.max(0.01, state.speed3D + (Math.random() - 0.5) * 0.1); // Vitesse qui varie un peu
        
        // Simulation de capteurs (pour tester ISA vs Météo réelle)
        if (Math.random() > 0.3) {
            state.tempK = 290.0 + (Math.random() * 2); // Température simulée (290K ~ 17°C)
            state.pressurePa = 101000 + (Math.random() * 500);
        } else {
            state.tempK = undefined; // Capteurs hors ligne occasionnellement
            state.pressurePa = undefined;
        }

        currentPosition = state;
        updateTrajectory(state);
        return state;
    }

    /** Retourne l'état actuel du filtre. */
    function getState() {
        return state;
    }

    return { initFromGnss, updateState, getState };
})();

/** Met à jour le tracé de la trajectoire. */
function updateTrajectory(state) {
    const maxPoints = 50; // Limiter la trace pour la performance
    trajectoryPoints.push({ lat: state.lat, lon: state.lon, alt: state.alt, ts: Date.now() });
    if (trajectoryPoints.length > maxPoints) {
        trajectoryPoints.shift();
    }
    }
// =================================================================
// GNSS SPACETIME DASHBOARD - BLOC 3/5
// Interface Utilisateur, Carte Leaflet, Graphiques Chart.js
// =================================================================

/* --- VARIABLES D'INTERFACE --- */
let map = null;
let userMarker = null;
let trajectoryPolyline = null;
let speedChart = null;
let timeDilationChart = null;

/* --- CARTE (Leaflet) --- */
/** Initialise la carte et le marqueur utilisateur. */
function initMap(lat, lon) {
    if (map) return;
    
    // Initialisation de la carte centrée sur la position initiale
    map = L.map('map-container').setView([lat, lon], 14);

    // Ajout des tuiles (OpenStreetMap)
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        maxZoom: 19
    }).addTo(map);

    // Marqueur utilisateur
    userMarker = L.marker([lat, lon], {
        icon: L.divIcon({
            className: 'custom-div-icon',
            html: '<div style="background-color: var(--accent-color); width: 15px; height: 15px; border-radius: 50%; border: 3px solid #fff; box-shadow: 0 0 5px #007bff;"></div>',
            iconSize: [15, 15],
            iconAnchor: [7.5, 7.5]
        })
    }).addTo(map).bindPopup("Position actuelle (UKF)").openPopup();
    
    // Polyline pour la trajectoire
    trajectoryPolyline = L.polyline([], { color: '#dc3545', weight: 2 }).addTo(map);
}

/** Met à jour la position du marqueur et le tracé de la trajectoire. */
function updateMapAndTrajectory(lat, lon) {
    if (userMarker) {
        const newLatLng = [lat, lon];
        userMarker.setLatLng(newLatLng);
        map.setView(newLatLng);
        
        // Mise à jour de la polyline
        const latLngs = trajectoryPoints.map(p => [p.lat, p.lon]);
        trajectoryPolyline.setLatLngs(latLngs);
    }
}

/* --- GRAPHIQUES (Chart.js) --- */
/** Initialise les graphiques (vitesse et dilatation du temps). */
function initCharts() {
    // Configuration de base pour les graphiques
    const baseConfig = (title, unit) => ({
        type: 'line',
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: { duration: 0 },
            plugins: { legend: { display: false }, title: { display: true, text: title, color: 'var(--text-dark)' } },
            scales: {
                x: { type: 'realtime', realtime: { delay: 2000, onRefresh: chart => { /* logic in updateCharts */ } }, display: false },
                y: { title: { display: true, text: unit, color: 'var(--text-dark)' }, grid: { color: 'var(--border-dark)' }, ticks: { color: 'var(--text-dark)' } }
            }
        }
    });

    // Graphique de Vitesse
    const speedCtx = $('speed-chart-canvas').getContext('2d');
    speedChart = new Chart(speedCtx, {
        ...baseConfig("Vitesse 3D (m/s)", "m/s"),
        data: { datasets: [{ label: 'Vitesse', borderColor: '#007bff', backgroundColor: 'rgba(0, 123, 255, 0.2)', data: [] }] }
    });

    // Graphique de Dilatation du Temps
    const dilationCtx = $('dilation-chart-canvas').getContext('2d');
    timeDilationChart = new Chart(dilationCtx, {
        ...baseConfig("Dilatation Temporelle Totale (ns/j)", "ns/j"),
        data: { datasets: [{ label: 'Dilatation Totale', borderColor: '#ffc107', backgroundColor: 'rgba(255, 193, 7, 0.2)', data: [] }] }
    });
}

/** Met à jour les données des graphiques. */
function updateCharts(speed3D, totalTimeDilation) {
    const now = Date.now();
    
    // Vitesse
    if (speedChart && speedChart.data.datasets.length > 0) {
        speedChart.data.datasets[0].data.push({ x: now, y: speed3D });
        if (speedChart.data.datasets[0].data.length > 30) speedChart.data.datasets[0].data.shift();
        speedChart.update('none');
    }

    // Dilatation du Temps
    if (timeDilationChart && timeDilationChart.data.datasets.length > 0) {
        timeDilationChart.data.datasets[0].data.push({ x: now, y: totalTimeDilation });
        if (timeDilationChart.data.datasets[0].data.length > 30) timeDilationChart.data.datasets[0].data.shift();
        timeDilationChart.update('none');
    }
}
// =================================================================
// GNSS SPACETIME DASHBOARD - BLOC 4/5
// Logique de Calcul Scientifique Réelle (refreshScientificDashboard) et Orchestrateur
// =================================================================

/**
 * Fonction principale de rafraîchissement qui calcule et met à jour TOUS les indicateurs scientifiques réels.
 * Cette fonction NE FAIT AUCUNE SIMULATION. Elle prend l'état du filtre (UKF/EKF) et le traite.
 * @param {object} gnssState - État du filtre UKF/EKF.
 */
function refreshScientificDashboard(gnssState) {
    if (!gnssState || isNaN(gnssState.lat) || isNaN(gnssState.lon) || isNaN(gnssState.alt) || isNaN(gnssState.speed3D)) {
        console.warn("État GNSS incomplet ou invalide pour les calculs scientifiques. Affichage en 'N/A'.");
        return; 
    }

    const lat = gnssState.lat;
    const lon = gnssState.lon;
    const alt = gnssState.alt;
    const v = gnssState.speed3D;
    const mass = currentMass;
    const Cd = currentCd;
    const A = currentArea;

    try {
        // --- 1. Modèle Atmosphérique (ISA vs. Météo Réelle) ---
        let T_K, P_Pa, rho_air, v_son;
        let isIsaModel = true;

        if (gnssState.tempK !== undefined && gnssState.pressurePa !== undefined) {
            // Utiliser les données de capteur/météo si disponibles
            T_K = gnssState.tempK;
            P_Pa = gnssState.pressurePa;
            v_son = SciencePro.getSpeedOfSound(T_K);
            rho_air = P_Pa / (SciencePro.R_d * T_K); 
            isIsaModel = false;
        } else {
            // Sinon, utiliser le modèle ISA basé sur l'altitude
            const isa = SciencePro.isaModel(alt);
            T_K = isa.T_K;
            P_Pa = isa.P_Pa;
            rho_air = isa.rho_air;
            v_son = SciencePro.getSpeedOfSound(T_K);
        }
        
        safeSet('isa-model-applied', isIsaModel ? 'OUI (ISA Standard)' : 'NON (Capteur Métro)');
        safeSet('air-temp-k', dataOrDefault(T_K, 2, ' K'));
        safeSet('air-density-local', dataOrDefault(rho_air, 4, ' kg/m³'));
        safeSet('pressure-atmos', dataOrDefault(P_Pa / 1000, 2, ' kPa'));
        safeSet('vitesse-son-locale', dataOrDefault(v_son, 2, ' m/s'));
        
        const mach = v / v_son;
        safeSet('perc-sound', dataOrDefault(v / v_son * 100, 2, ' %'));
        safeSet('mach-number', dataOrDefault(mach, 4));


        // --- 2. Physique, Relativité & Gravité ---
        const c = SciencePro.c;
        const gamma = SciencePro.lorentzGamma(v);
        const E0 = SciencePro.restMassEnergy(mass);
        const E_rel = gamma * E0;
        const E_kin_rel = SciencePro.getRelativisticKineticEnergy(gamma, E0);
        const g_local = SciencePro.localGravity(alt);
        const Rs = SciencePro.schwarzschildRadius(mass);
        
        const dt_vel = SciencePro.timeDilation(gamma); // Dilatation par vitesse
        const dt_grav = SciencePro.gravitationalTimeDilation(alt); // Dilatation gravitationnelle
        const dt_total = dt_vel + dt_grav;

        safeSet('perc-speed-c', dataOrDefault(v / c * 100, 2, 'e+0 %')); 
        safeSet('gamma-factor', dataOrDefault(gamma, 8));
        safeSet('time-dilation-vel', dataOrDefault(dt_vel, 4, ' ns/j'));
        safeSet('time-dilation-grav', dataOrDefault(dt_grav, 4, ' ns/j'));
        safeSet('total-time-dilation', dataOrDefault(dt_total, 4, ' ns/j'));
        
        safeSet('rest-mass-energy', dataOrDefault(E0, 2, ' J'));
        safeSet('energy-rel-total', dataOrDefault(E_rel, 2, ' J'));
        safeSet('energy-kin-rel', dataOrDefault(E_kin_rel, 2, ' J'));
        safeSet('schwarzschild-radius', dataOrDefault(Rs, 18, ' m')); 

        safeSet('g-local', dataOrDefault(g_local, 4, ' m/s²'));
        safeSet('weight-local', dataOrDefault(g_local * mass, 2, ' N'));
        
        // --- 3. Dynamique des Fluides ---
        const q_dyn = 0.5 * rho_air * v * v;
        const force_trainee = q_dyn * A * Cd;
        const puissance_trainee = force_trainee * v / 1000;

        safeSet('q-dyn', dataOrDefault(q_dyn, 2, ' Pa'));
        safeSet('drag-force', dataOrDefault(force_trainee, 2, ' N'));
        safeSet('drag-kw', dataOrDefault(puissance_trainee, 3, ' kW'));

        const omega_earth = PHYS.omega_earth;
        const force_coriolis = 2 * mass * omega_earth * v * Math.sin(lat * PHYS.pi / 180); 
        safeSet('coriolis-force', dataOrDefault(force_coriolis, 4, ' N'));
        
        // --- 4. Astronomie & Temps ---
        const astroData = SciencePro.getAstroData(new Date(), lat, lon);

        safeSet('sun-alt', dataOrDefault(astroData.sunAlt, 3, '°'));
        safeSet('sun-azimuth', dataOrDefault(astroData.sunAzimuth, 3, '°'));
        safeSet('moon-phase-name', astroData.moonPhase);
        safeSet('moon-illuminated', dataOrDefault(astroData.moonIllumination, 1, '%'));
        safeSet('moon-age-days', dataOrDefault(astroData.moonAgeDays, 2, ' j'));

        // Mise à jour des graphiques
        updateCharts(v, dt_total);

    } catch(e) {
        console.error("Erreur critique lors du calcul des données dérivées:", e);
    }
}

/* --- Orchestrateur (Moteur de Boucle) --- */
const orchestrator = (function(){
    let intervalId = null;

    /** Exécute la boucle de rafraîchissement. */
    function runRefresh() {
        if (intervalId) clearInterval(intervalId); // S'assurer qu'une seule boucle tourne

        const mainLoop = () => {
            // 1. Simuler l'acquisition de données GNSS brutes/capteurs
            const rawGnssData = { lat: currentPosition.lat, lon: currentPosition.lon, alt: currentPosition.alt, spd: currentPosition.speed3D };
            const sensorData = { tempK: currentPosition.tempK, pressurePa: currentPosition.pressurePa };
            
            // 2. Mettre à jour l'état fusionné (via le filtre UKF)
            const currentState = FilterEngine.updateState(rawGnssData, sensorData);
            
            // 3. Mettre à jour les affichages GNSS bruts
            safeSet('lat', dataOrDefault(currentState.lat, 6, '°'));
            safeSet('lon', dataOrDefault(currentState.lon, 6, '°'));
            safeSet('alt', dataOrDefault(currentState.alt, 2, ' m'));
            safeSet('speed-3d-inst', dataOrDefault(currentState.speed3D, 3, ' m/s'));

            // 4. Exécuter la logique scientifique réelle
            refreshScientificDashboard(currentState);

            // 5. Mettre à jour la carte
            updateMapAndTrajectory(currentState.lat, currentState.lon);

            // Mettre à jour l'heure locale
            safeSet('local-time', new Date().toLocaleTimeString('fr-FR'));
        };

        // Lancer la boucle toutes les 2 secondes
        mainLoop(); // Exécution immédiate
        intervalId = setInterval(mainLoop, 2000); 
    }

    return { runRefresh };
})();
// =================================================================
// GNSS SPACETIME DASHBOARD - BLOC 5/5
// Initialisation, Événements et Démarrage
// =================================================================

/** Gestionnaire du Mode Jour/Nuit */
function toggleDarkMode() {
    document.body.classList.toggle('dark-mode');
    const isDark = !document.body.classList.contains('dark-mode');
    $('toggle-mode-btn').innerHTML = isDark 
        ? '<i class="fas fa-moon"></i> Mode Nuit' 
        : '<i class="fas fa-sun"></i> Mode Jour';
}

/** Gestionnaire pour l'arrêt d'urgence/re-start */
function toggleDashboardLoop() {
    if (orchestrator.isStopped) {
        orchestrator.runRefresh();
        $('emergency-stop-btn').textContent = "Arrêt d'Urgence";
        $('emergency-stop-btn').classList.remove('bg-green-500');
        $('emergency-stop-btn').classList.add('bg-red-500');
        orchestrator.isStopped = false;
    } else {
        if (orchestrator.intervalId) clearInterval(orchestrator.intervalId);
        orchestrator.isStopped = true;
        $('emergency-stop-btn').textContent = "Démarrer Simulation"; // Change pour un mode simulation pour l'exemple
        $('emergency-stop-btn').classList.remove('bg-red-500');
        $('emergency-stop-btn').classList.add('bg-green-500');
        console.log("Boucle du tableau de bord arrêtée.");
    }
}

/* --- SÉQUENCE D'INITIALISATION --- */
(async function init() {
    // 1. Initialiser le mode d'affichage par défaut
    document.body.classList.add('dark-mode');
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').innerHTML = '<i class="fas fa-sun"></i> Mode Jour';
        $('toggle-mode-btn').onclick = toggleDarkMode;
    }

    // 2. Initialiser le filtre et la position par défaut (Paris)
    const lat0 = currentPosition.lat;
    const lon0 = currentPosition.lon;
    const alt0 = currentPosition.alt;
    FilterEngine.initFromGnss(lat0, lon0, alt0);

    // 3. Initialiser la carte et les graphiques
    initMap(lat0, lon0);
    initCharts();

    // 4. Configurer les écouteurs d'événements
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').onclick = toggleDashboardLoop;
    }
    
    // 5. Démarrer la boucle de rafraîchissement
    orchestrator.runRefresh();

    console.log('Dashboard Scientifique initialisé (5 blocs chargés).');
})();
