// =====================================================================================
// DEBUT DU CODE JAVASCRIPT COMPLETÉ
// =====================================================================================

// ========================
// ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = []; 
let distanceTotale = 0;
let tempsDebut = null; 
let intervalleTemps = null; 
let modeSouterrainActif = false; 
// Coordonnées Tour Eiffel par défaut (48.8584, 2.2945)
let targetCoords = { latitude: 48.8584, longitude: 2.2945 }; 
let deviceOrientationListener = null; 

// Constantes physiques
const VITESSE_LUMIERE = 299792458; // m/s
const VITESSE_SON = 343; // m/s (approx. à 20°C)
const R_TERRE = 6371e3; // Rayon de la Terre en mètres
const ANNEE_LUMIERE_SECONDES = 3600 * 24 * 365.25;
const MASSE_EXPLORATEUR = 70; // kg (pour l'énergie cinétique)
const INTENSITE_MIN_UT = 30; // Min field intensity for simulation (Equator)
const INTENSITE_MAX_UT = 60; // Max field intensity for simulation (Poles)

// Capteurs et Coordonnées par défaut
let capteursEtat = {
  niveauBulle: '--',
  lumiere: '--',
  son: '--',
  magnetisme: '--',
  pression: null,
  boussole: null 
};
const DEFAULT_LONGITUDE = 2.3522; 
const DEFAULT_LATITUDE = 48.8566; 

// ========================
// UTILS & MATH
// ========================
function safeSetText(id, text) { 
  const e = document.getElementById(id); 
  if (e) e.textContent = text; 
}
function safeDisplay(id, display) {
    const e = document.getElementById(id);
    if (e) e.style.display = display;
}
function toRadians(degrees) { return degrees * Math.PI / 180; }
function toDegrees(radians) { return radians * 180 / Math.PI; }
function formatHours(h) {
    const hours = Math.floor(h);
    const minutes = Math.floor((h - hours) * 60);
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
}

// ========================
// CALCULS DE BASE (GPS/VITESSE)
// ========================

function calculerDistanceHaversine(p1, p2) {
  const φ1 = toRadians(p1.latitude);
  const φ2 = toRadians(p2.latitude);
  const Δφ = toRadians(p2.latitude - p1.latitude);
  const Δλ = toRadians(p2.longitude - p1.longitude);
  const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R_TERRE * c; 
}

function miseAJourVitesse(pos) {
    if (modeSouterrainActif) {
        return; 
    }
    
    const gps = {
        latitude: pos.coords.latitude,
        longitude: pos.coords.longitude,
        altitude: pos.coords.altitude,
        accuracy: pos.coords.accuracy,
        timestamp: pos.timestamp,
        speed: pos.coords.speed, 
        heading: pos.coords.heading 
    };

    let vitesse_ms = 0;
    
    if (positionPrecedente) {
        const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
        const distance_parcourue = calculerDistanceHaversine(gps, positionPrecedente);
        
        distanceTotale += distance_parcourue;
        
        // Utiliser la vitesse fournie par le GPS si elle est fiable (et inclut la composante verticale)
        if (typeof gps.speed === 'number' && gps.speed >= 0 && gps.speed < 1000) { 
            vitesse_ms = gps.speed; 
        } else if (dt > 0) {
            // Calculer la vitesse à partir de la distance et du temps (principalement horizontale)
            vitesse_ms = distance_parcourue / dt; 
        }
    }
    
    positionPrecedente = gps;
    const vitesse_kmh = vitesse_ms * 3.6;

    if (vitesse_kmh >= 0) { 
        vitesseMax = Math.max(vitesseMax, vitesse_kmh);
        vitesses.push(vitesse_kmh);
        if (vitesses.length > 30) vitesses.shift();

        afficherVitesse(vitesse_kmh);
        afficherDistance();
        afficherTemps();
        afficherGrandeurs(vitesse_ms);
        
        if (positionPrecedente) {
            updateNavigation(positionPrecedente, targetCoords);
            updateCompass(gps.heading); 
        }
    }

    afficherGPS(gps);
}

function afficherVitesse(v_kmh) {
  const mps = v_kmh / 3.6;
  const mmps = mps * 1000;
  const moyenne_kmh = vitesses.length ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length : 0;

  safeSetText('vitesse', `Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
  safeSetText('pourcentage', `% Lumière : ${(mps / VITESSE_LUMIERE * 100).toExponential(2)}% | % Son : ${(mps / VITESSE_SON * 100).toFixed(2)}%`);
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  const m = distanceTotale;
  const mm = m * 1000;
  const secLumiere = m / VITESSE_LUMIERE;
  const al = secLumiere / ANNEE_LUMIERE_SECONDES;

  safeSetText('distance', `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm`);
  safeSetText('distance-cosmique', `Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
  if (!tempsDebut) return;
  const tempsEcoule = (Date.now() - tempsDebut) / 1000;
  safeSetText('temps', `Temps : ${tempsEcoule.toFixed(2)} s`);
}

function afficherGPS(g) {
  safeSetText('gps', `GPS : Lat : ${g.latitude.toFixed(6)} | Lon : ${g.longitude.toFixed(6)} | Alt : ${g.altitude?.toFixed(0)??'--'} m | Préc. : ${g.accuracy?.toFixed(0)??'--'} m`);
}

// ========================
// GRANDEURS PHYSIQUES & MAGNÉTISME
// ========================

function estimerIntensiteMagnetique(latitude) {
    const absLat = Math.abs(latitude);
    const normalizedLat = absLat / 90; 
    const intensite = INTENSITE_MIN_UT + (INTENSITE_MAX_UT - INTENSITE_MIN_UT) * normalizedLat;
    return intensite.toFixed(2);
}

function afficherGrandeurs(vitesse_ms) {
    const P_atm_mer = 1013.25; 
    let P_estimee = P_atm_mer;

    // Formule barométrique simplifiée pour l'altitude
    if (positionPrecedente && positionPrecedente.altitude !== null) {
        const alt_m = positionPrecedente.altitude;
        P_estimee = P_atm_mer * Math.pow(1 - (0.0065 * alt_m) / 288.15, 5.257); 
    }
    capteursEtat.pression = P_estimee.toFixed(2);
    
    let E_cinetique = '--';
    if (typeof vitesse_ms === 'number' && vitesse_ms >= 0) {
        E_cinetique = (0.5 * MASSE_EXPLORATEUR * vitesse_ms * vitesse_ms).toFixed(2);
    }

    safeSetText('grandeurs', `Pression Est. : ${capteursEtat.pression} hPa | Énergie cinétique Est. : ${E_cinetique} J`);
}

// ========================
// CARTE & NAVIGATION
// ========================

function getBearing(p1, p2) {
    const φ1 = toRadians(p1.latitude);
    const λ1 = toRadians(p1.longitude);
    const φ2 = toRadians(p2.latitude);
    const λ2 = toRadians(p2.longitude);

    const y = Math.sin(λ2 - λ1) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) -
              Math.sin(φ1) * Math.cos(φ2) * Math.cos(λ2 - λ1);
    const bearing = toDegrees(Math.atan2(y, x));
    return (bearing + 360) % 360; 
}

function updateNavigation(currentPos, targetPos) {
    const bearing = getBearing(currentPos, targetPos);
    const distance_m = calculerDistanceHaversine(currentPos, targetPos);
    const distance_km = distance_m / 1000;

    safeSetText('bearing-display', `Relèvement vers la cible : ${bearing.toFixed(1)}° | Distance : ${distance_km.toFixed(2)} km`);
}

function updateCompass(heading) {
    let heading_deg = '--';
    if (typeof heading === 'number' && heading !== null) {
        heading_deg = heading.toFixed(1);
    } else if (capteursEtat.boussole) {
        heading_deg = capteursEtat.boussole.toFixed(1);
    }
    
    safeSetText('compass-display', `Boussole (Nord Vrai) : ${heading_deg}°`);
}

function handleOrientation(event) {
    let alpha = event.alpha; 
    if (event.webkitCompassHeading) { 
        alpha = event.webkitCompassHeading;
    }
    if (typeof alpha === 'number') {
        capteursEtat.boussole = (360 - alpha) % 360; 
        
        if (!positionPrecedente || positionPrecedente.heading === null) {
            updateCompass(capteursEtat.boussole);
        }
    }
}

// ========================
// CALCULS ASTRONOMIQUES AVANCÉS
// ========================

function angleToTime(angle_deg) {
    let angle = angle_deg % 360;
    if (angle < 0) angle += 360;
    
    const total_sec = (angle * 24 * 3600) / 360;
    const hours = Math.floor(total_sec / 3600);
    const minutes = Math.floor((total_sec % 3600) / 60);
    const seconds = Math.floor(total_sec % 60);
    
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
}

function calculerHeuresSolaires(now, latitude, longitude) {
    const Jd = (now.getTime() / (24 * 60 * 60 * 1000)) + 2440587.5; 
    const T = (Jd - 2451545.0) / 36525; 

    const M = 357.52911 + 35999.05029 * T - 0.0001537 * T * T; 
    const M_rad = toRadians(M % 360);
    const L0 = 280.46646 + 36000.76983 * T + 0.0003032 * T * T; 
    const L0_mod = L0 % 360; 
    const C = (1.9146 - 0.004817 * T) * Math.sin(M_rad) + (0.01994 - 0.000101 * T) * Math.sin(2 * M_rad); 
    const lambda = L0_mod + C; 
    const lambda_rad = toRadians(lambda);
    const epsilon = 23.439291 - 0.013004 * T; 
    const epsilon_rad = toRadians(epsilon);
    const alpha_rad = Math.atan2(Math.cos(epsilon_rad) * Math.sin(lambda_rad), Math.cos(lambda_rad)); 
    const alpha_deg = toDegrees(alpha_rad);
    const alpha_norm = alpha_deg < 0 ? alpha_deg + 360 : alpha_deg;

    let EqT_deg = L0_mod - alpha_norm;
    if (EqT_deg > 180) EqT_deg -= 360; 
    if (EqT_deg < -180) EqT_deg += 360;

    const EqT_sec = EqT_deg * 240; 
    
    const UTC_h = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    const HSML_deg = (UTC_h * 15 + longitude) % 360;
    const HSML_norm = HSML_deg < 0 ? HSML_deg + 360 : HSML_deg;
    
    const HSLV_deg = (HSML_norm + EqT_deg) % 360; 
    const HSLV_norm = HSLV_deg < 0 ? HSLV_deg + 360 : HSLV_deg;

    safeSetText('heure-vraie', `Heure Solaire Vraie : ${angleToTime(HSLV_norm)}`);
    safeSetText('heure-moyenne', `Heure Solaire Moyenne : ${angleToTime(HSML_norm)}`);
    safeSetText('equation-temps', `Équation du Temps : ${EqT_sec.toFixed(2)} s`);

    const delta_rad = Math.asin(Math.sin(epsilon_rad) * Math.sin(lambda_rad)); 
    const H0_soleil = toRadians(-0.83); 
    const phi_rad = toRadians(latitude);

    const cosH_soleil = (Math.sin(H0_soleil) - Math.sin(phi_rad) * Math.sin(delta_rad)) / (Math.cos(phi_rad) * Math.cos(delta_rad));
    
    if (cosH_soleil <= 1 && cosH_soleil >= -1) {
        const H_soleil_deg = toDegrees(Math.acos(cosH_soleil)); 
        const H_soleil_h = H_soleil_deg / 15;
        
        const culmination_h_HSML = 12; 
        
        const lever_h_HSML = (culmination_h_HSML - H_soleil_h + 24) % 24;
        const coucher_h_HSML = (culmination_h_HSML + H_soleil_h) % 24;
        
        const EqT_h = EqT_deg / 15; 
        const lever_h_HSLV = (lever_h_HSML + EqT_h + 24) % 24;
        const coucher_h_HSLV = (coucher_h_HSML + EqT_h + 24) % 24;

        safeSetText('soleil-lever', `Lever Soleil HSLV : ${formatHours(lever_h_HSLV)} | HSML : ${formatHours(lever_h_HSML)}`);
        safeSetText('soleil-coucher', `Coucher Soleil HSLV : ${formatHours(coucher_h_HSLV)} | HSML : ${formatHours(coucher_h_HSML)}`);
    } else {
        safeSetText('soleil-lever', (latitude >= 66.5) ? 'Lever Soleil HSLV : Jour Polaire' : 'Lever Soleil HSLV : Nuit Polaire');
        safeSetText('soleil-coucher', (latitude >= 66.5) ? 'Coucher Soleil HSLV : Nuit Polaire' : 'Coucher Soleil HSLV : Jour Polaire');
    }

    const lune_results = calculerLune(Jd);
    safeSetText('lune-phase', `Phase Lune : ${lune_results.phase}`);
    safeSetText('lune-lever', `Lever Lune HSLV : -- | HSML : --`); 
    safeSetText('lune-coucher', `Coucher Lune HSLV : -- | HSML : --`); 
    safeSetText('lune-culmination', `Culmination Lune : --`);
    safeSetText('lune-mag', `Magnitude Lune : ${lune_results.mag}`);

    drawMinecraftClock(HSLV_norm / 15, lune_results.phase); 
}

function calculerLune(Jd) {
    const N = Jd - 2451550.09765; 
    const age = N % 29.530588853; 
    const phase_fraction = age / 29.530588853;

    let phase_nom = '';
    if (phase_fraction < 0.03 || phase_fraction > 0.97) phase_nom = 'Nouvelle Lune';
    else if (phase_fraction < 0.25) phase_nom = 'Croissant Jeune';
    else if (phase_fraction < 0.28) phase_nom = 'Premier Quartier';
    else if (phase_fraction < 0.50) phase_nom = 'Gibbeuse Croissante';
    else if (phase_fraction < 0.53) phase_nom = 'Pleine Lune';
    else if (phase_fraction < 0.75) phase_nom = 'Gibbeuse Décroissante';
    else if (phase_fraction < 0.78) phase_nom = 'Dernier Quartier';
    else phase_nom = 'Croissant Vieillissant';

    return {
        phase: phase_nom,
        mag: '--' 
    };
}

function activerHorlogeSolaire(latitude, longitude) {
    let currentLatitude = latitude;
    let currentLongitude = longitude;
    
    if (window.solarInterval) clearInterval(window.solarInterval);
    
    calculerHeuresSolaires(new Date(), currentLatitude, currentLongitude);

    window.solarInterval = setInterval(() => {
        if (positionPrecedente) {
            currentLatitude = positionPrecedente.latitude;
            currentLongitude = positionPrecedente.longitude;
        }

        const now = new Date();
        calculerHeuresSolaires(now, currentLatitude, currentLongitude);
    }, 60000);
}


// ========================
// MONTRE MINECRAFT
// ========================
function drawMinecraftClock(HSLV_hour, phase_nom) {
    const canvas = document.getElementById('minecraft-clock');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const size = 100;
    const center = size / 2;
    const radius = size * 0.4; 
    
    const angle_deg = ((HSLV_hour / 24 * 360) - 90) % 360; 
    const angle_rad = toRadians(angle_deg); 

    ctx.clearRect(0, 0, size, size);
    ctx.strokeStyle = '#00ffcc';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, 2 * Math.PI);
    ctx.stroke();

    let is_day = HSLV_hour > 6 && HSLV_hour < 18;
    let time_status = is_day ? 'Jour' : 'Nuit';

    const x = center + radius * Math.cos(angle_rad);
    const y = center + radius * Math.sin(angle_rad);
    
    ctx.beginPath();
    ctx.arc(x, y, 8, 0, 2 * Math.PI);

    if (is_day) {
        ctx.fillStyle = '#ffd700'; 
    } else {
        ctx.fillStyle = '#ffffff'; 
    }
    ctx.fill();
    
    const month = new Date().getMonth();
    let season = '';
    if (month >= 2 && month <= 4) season = 'Printemps';
    else if (month >= 5 && month <= 7) season = 'Été';
    else if (month >= 8 && month <= 10) season = 'Automne';
    else season = 'Hiver';

    safeSetText('minecraft-time-display', `${time_status} | Saison : ${season} | ${phase_nom}`);
}

// ========================
// GESTION DU COCKPIT (GÉNÉRAL)
// ========================

function demarrerCockpit() {
    if (watchId !== null) return; 
    
    if (modeSouterrainActif) {
         toggleModeSouterrain();
         return; 
    }

    if (tempsDebut === null) {
        tempsDebut = Date.now(); 
        distanceTotale = 0;
        vitesseMax = 0;
        vitesses = [];
    }
    
    const options = {
        enableHighAccuracy: true,
        timeout: 5000,
        maximumAge: 0
    };

    watchId = navigator.geolocation.watchPosition(
        miseAJourVitesse, 
        (error) => {
            console.error("Erreur Géolocalisation : ", error);
            safeSetText('gps', `GPS : Erreur (${error.code}) : ${error.message}`);
        }, 
        options
    );
    
    if (intervalleTemps === null) {
        intervalleTemps = setInterval(updateSystemClock, 1000);
    }
    
    activerHorlogeSolaire(positionPrecedente?.latitude || DEFAULT_LATITUDE, positionPrecedente?.longitude || DEFAULT_LONGITUDE);
    
    document.getElementById('marche').classList.add('pulsation');
}

function arreterCockpit() {
    if (watchId !== null) {
        navigator.geolocation.clearWatch(watchId);
        watchId = null;
    }
    document.getElementById('marche').classList.remove('pulsation');
    safeSetText('vitesse', 'Vitesse instantanée : **ARRÊT**');
}

function resetCockpit() {
    arreterCockpit();
    
    positionPrecedente = null;
    vitesseMax = 0;
    vitesses = []; 
    distanceTotale = 0;
    tempsDebut = null; 
    
    safeSetText('temps', 'Temps : 0.00 s');
    safeSetText('vitesse', 'Vitesse instantanée : -- km/h');
    safeSetText('vitesse-moy', 'Vitesse moyenne : -- km/h');
    safeSetText('vitesse-max', 'Vitesse max : -- km/h');
    safeSetText('vitesse-ms', 'Vitesse : -- m/s | -- mm/s');
    safeSetText('pourcentage', '% Lumière : --% | % Son : --%');
    safeSetText('distance', 'Distance : -- km | -- m | -- mm');
    safeSetText('distance-cosmique', 'Distance cosmique : -- s lumière | -- al');
    safeSetText('gps', 'GPS : --');
    safeSetText('bearing-display', 'Relèvement vers la cible : --° | Distance : -- km');
    safeSetText('compass-display', 'Boussole (Nord Vrai) : --°');
    safeSetText('grandeurs', 'Pression Est. : -- hPa | Énergie cinétique Est. : -- J');
}

// ========================
// GESTION DES BOUTONS & HORLOGES
// ========================

function toggleModeSouterrain() {
    modeSouterrainActif = !modeSouterrainActif;
    const button = document.getElementById('toggle-souterrain');
    const indicator = document.getElementById('mode-souterrain-indicator');

    if (modeSouterrainActif) {
        if (watchId !== null) arreterCockpit(); 
        safeDisplay('mode-souterrain-indicator', 'block');
        button.textContent = '✅ Mode Souterrain : ON';
        button.style.background = '#004400';
        button.style.borderColor = '#00ff45';
        safeSetText('gps', 'GPS : **COUPÉ**');
    } else {
        safeDisplay('mode-souterrain-indicator', 'none');
        button.textContent = '🚫 Mode Souterrain : OFF';
        button.style.background = '#440000';
        button.style.borderColor = '#ff4500';
        
        if (tempsDebut !== null) {
            demarrerCockpit();
        }
    }
}

function toggleRituel() {
    const body = document.body;
    const button = document.getElementById('toggle-rituel');

    if (body.classList.contains('rituel-off')) {
        body.classList.remove('rituel-off');
        button.textContent = '✅ Rituel cosmique : ON';
        document.getElementById('marche').classList.add('pulsation');
    } else {
        body.classList.add('rituel-off');
        button.textContent = '❌ Rituel cosmique : OFF';
        document.getElementById('marche').classList.remove('pulsation');
    }
}

function setTargetCoords() {
    const input = document.getElementById('target-coord').value;
    const parts = input.split(',').map(p => parseFloat(p.trim()));
    
    if (parts.length === 2 && !isNaN(parts[0]) && !isNaN(parts[1])) {
        targetCoords.latitude = parts[0];
        targetCoords.longitude = parts[1];
        
        if (positionPrecedente) {
            updateNavigation(positionPrecedente, targetCoords);
        }
        alert(`Nouvelle cible verrouillée : Lat ${targetCoords.latitude.toFixed(4)}, Lon ${targetCoords.longitude.toFixed(4)}`);
    } else {
        alert('Format de coordonnées invalide. Utilisez "Lat, Lon" (ex: 48.8584, 2.2945)');
    }
}

function updateSystemClock() {
    const now = new Date();
    safeSetText('horloge', `⏰ ${now.toLocaleTimeString('fr-FR', {hour12: false})} (Système)`);
    safeSetText('horloge-atomique', `Heure atomique (UTC) : ${now.getUTCHours().toString().padStart(2, '0')}:${now.getUTCMinutes().toString().padStart(2, '0')}:${now.getUTCSeconds().toString().padStart(2, '0')}`);
    
    updateCapteursSimules();
}

function updateCapteursSimules() 
