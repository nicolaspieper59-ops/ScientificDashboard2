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
let deviceOrientationListener = null; // Pour la boussole

// Constantes physiques
const VITESSE_LUMIERE = 299792458; // m/s
const VITESSE_SON = 343; // m/s (approx. à 20°C)
const R_TERRE = 6371e3; // Rayon de la Terre en mètres
const ANNEE_LUMIERE_SECONDES = 3600 * 24 * 365.25;
const MASSE_EXPLORATEUR = 70; // kg (pour l'énergie cinétique)

// Capteurs et Coordonnées par défaut
let capteursEtat = {
  niveauBulle: '--',
  lumiere: '--',
  son: '--',
  magnetisme: '--',
  pression: null // Ajout pour la pression
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
function formatHoursSeconds(h) {
    const hours = Math.floor(h);
    const minutes = Math.floor((h - hours) * 60);
    const seconds = Math.floor(((h - hours) * 60 - minutes) * 60);
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
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
  return R_TERRE * c; // Distance en mètres
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
        speed: pos.coords.speed, // m/s
        heading: pos.coords.heading // degrés
    };

    let vitesse_ms = 0;
    
    if (positionPrecedente) {
        const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
        const distance_parcourue = calculerDistanceHaversine(gps, positionPrecedente);
        
        distanceTotale += distance_parcourue;
        
        if (typeof gps.speed === 'number' && gps.speed >= 0 && gps.speed < 1000) { 
            vitesse_ms = gps.speed; 
        } else if (dt > 0) {
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
            updateCompass(gps.heading); // Utilise le heading natif si disponible
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
// GRANDEURS PHYSIQUES
// ========================

function afficherGrandeurs(vitesse_ms) {
    const P_atm_mer = 1013.25; // hPa
    let P_estimee = P_atm_mer;

    // Si on a l'altitude, on peut estimer la pression (Formule barométrique simplifiée)
    if (positionPrecedente && positionPrecedente.altitude) {
        // Pression = P_0 * exp(-g * M * (h - h_0) / (R * T))
        // Simplification: 1hPa tous les 8m
        const alt_m = positionPrecedente.altitude;
        P_estimee = P_atm_mer - (alt_m / 8.3); // Approximation simple
    }
    capteursEtat.pression = P_estimee.toFixed(2);
    
    // Énergie cinétique (pour un objet de 70kg)
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
    const heading_deg = heading !== null ? heading.toFixed(1) : '--';
    safeSetText('compass-display', `Boussole (Nord Vrai) : ${heading_deg}°`);
}

// Gère la boussole utilisant l'API DeviceOrientation (si le heading natif GPS n'est pas dispo)
function handleOrientation(event) {
    let alpha = event.alpha; // Z-axis rotation (0 to 360)
    if (event.webkitCompassHeading) { // iOS
        alpha = event.webkitCompassHeading;
    }
    if (typeof alpha === 'number') {
        updateCompass(360 - alpha); // Alpha est souvent l'inverse
    }
}

// ========================
// CALCULS ASTRONOMIQUES AVANCÉS
// ========================

function calculerHeuresSolaires(now, latitude, longitude) {
    const Jd = (now.getTime() / (24 * 60 * 60 * 1000)) + 2440587.5;
    const T = (Jd - 2451545.0) / 36525;

    // --- Calcul EqT, HSLV, HSML ---
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

    // --- Calcul Déclinaison du Soleil (δ) ---
    const delta_rad = Math.asin(Math.sin(epsilon_rad) * Math.sin(lambda_rad));
    const delta_deg = toDegrees(delta_rad);

    // --- Lever/Coucher Soleil (H0 = -0.83°) ---
    const H0_soleil = toRadians(-0.83); 
    const phi_rad = toRadians(latitude);

    const cosH_soleil = (Math.sin(H0_soleil) - Math.sin(phi_rad) * Math.sin(delta_rad)) / (Math.cos(phi_rad) * Math.cos(delta_rad));
    
    if (cosH_soleil <= 1 && cosH_soleil >= -1) {
        const H_soleil_deg = toDegrees(Math.acos(cosH_soleil)); 
        const H_soleil_h = H_soleil_deg / 15;
        
        const culmination_h_HSLV = 12 - (EqT_sec / 3600); 
        const culmination_h_HSML = 12;

        const lever_h_HSLV = (culmination_h_HSLV - H_soleil_h + 24) % 24;
        const coucher_h_HSLV = (culmination_h_HSLV + H_soleil_h) % 24;
        const lever_h_HSML = (culmination_h_HSML - H_soleil_h + 24) % 24;
        const coucher_h_HSML = (culmination_h_HSML + H_soleil_h) % 24;

        safeSetText('soleil-lever', `Lever Soleil HSLV : ${formatHours(lever_h_HSLV)} | HSML : ${formatHours(lever_h_HSML)}`);
        safeSetText('soleil-coucher', `Coucher Soleil HSLV : ${formatHours(coucher_h_HSLV)} | HSML : ${formatHours(coucher_h_HSML)}`);
    } else {
        safeSetText('soleil-lever', 'Lever Soleil HSLV : Jour Polaire');
        safeSetText('soleil-coucher', 'Coucher Soleil HSLV : Nuit Polaire');
    }

    // --- Calculs Lunaires Simplifiés ---
    const lune_results = calculerLune(Jd, latitude, longitude);
    safeSetText('lune-phase', `Phase Lune : ${lune_results.phase}`);
    safeSetText('lune-lever', `Lever Lune HSLV : ${lune_results.lever || '--'} | HSML : ${lune_results.lever || '--'}`); // Utilisation simplifiée
    safeSetText('lune-coucher', `Coucher Lune HSLV : ${lune_results.coucher || '--'} | HSML : ${lune_results.coucher || '--'}`); // Utilisation simplifiée
    safeSetText('lune-culmination', `Culmination Lune : ${lune_results.culmination || '--'}`);
    safeSetText('lune-mag', `Magnitude Lune : ${lune_results.mag || '--'}`);

    // --- Montre Minecraft (Mise à jour) ---
    drawMinecraftClock(HSLV_norm / 15, lune_results.phase); // HSLV en heures (0-24)
}

function calculerLune(Jd, latitude, longitude) {
    // Ceci est une approximation très simple, le calcul complet est extrêmement lourd en JS.
    const K = Math.floor((Jd - 2451550.1) / 29.530588853); // Nombre de jours lunaires
    const T = (Jd - 2451545.0) / 36525;

    // Phase Lunaire (Approximation du NOAA)
    const T0 = (Jd - 2451550.09765) / 36525.0;
    const L = 218.316 + 481267.881 * T0; // Longitude moyenne de la Lune
    const S = 135.963 + 477198.868 * T0; // Anomalie moyenne de la Lune
    const I = 297.85 + 445267.112 * T0; // Longitude moyenne du Soleil
    const D = 350.738 + 445267.112 * T0; // Distance angulaire moyenne
    
    const phase_rad = toRadians((L - S) % 360);
    const phase_fraction = (L - S) % 360 / 360; // 0=Nouvelle Lune, 0.5=Pleine Lune

    let phase_nom = '';
    if (phase_fraction < 0.05 || phase_fraction > 0.95) phase_nom = 'Nouvelle Lune';
    else if (phase_fraction < 0.22) phase_nom = 'Croissant Jeune';
    else if (phase_fraction < 0.28) phase_nom = 'Premier Quartier';
    else if (phase_fraction < 0.45) phase_nom = 'Gibbeuse Croissante';
    else if (phase_fraction < 0.55) phase_nom = 'Pleine Lune';
    else if (phase_fraction < 0.72) phase_nom = 'Gibbeuse Décroissante';
    else if (phase_fraction < 0.78) phase_nom = 'Dernier Quartier';
    else phase_nom = 'Croissant Vieillissant';

    // Lever/Coucher/Culmination sont trop complexes sans une librairie complète.
    return {
        phase: phase_nom,
        mag: '--', // Magnitude non calculée
        lever: '--',
        coucher: '--',
        culmination: '--'
    };
}

function activerHorlogeSolaire(latitude, longitude) {
    let currentLatitude = latitude;
    let currentLongitude = longitude;
    
    // Initialisation
    calculerHeuresSolaires(new Date(), currentLatitude, currentLongitude);

    // Boucle de mise à jour toutes les 1000ms
    setInterval(() => {
        if (positionPrecedente) {
            currentLatitude = positionPrecedente.latitude;
            currentLongitude = positionPrecedente.longitude;
        }

        const now = new Date();
        calculerHeuresSolaires(now, currentLatitude, currentLongitude);
    }, 1000);
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
    const radius = size * 0.45;
    
    // L'angle est basé sur l'heure HSLV (0-24h). 6h = Lever (0°), 18h = Coucher (180°)
    const angle_deg = (HSLV_hour / 24 * 360 + 90) % 360; // Décalé de 90° pour commencer le jour
    const angle_rad = toRadians(angle_deg); 

    ctx.clearRect(0, 0, size, size);
    ctx.strokeStyle = '#00ffcc';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, 2 * Math.PI);
    ctx.stroke();

    // Cycle Jour/Nuit (Couleur et affichage)
    let is_day = HSLV_hour > 6 && HSLV_hour < 18;
    let time_status = is_day ? 'Jour' : 'Nuit';

    // 1. Position du Soleil/Lune
    const x = center + radius * Math.cos(angle_rad);
    const y = center + radius * Math.sin(angle_rad);
    
    ctx.beginPath();
    ctx.arc(x, y, 8, 0, 2 * Math.PI);

    // Couleur en fonction du cycle
    if (is_day) {
        ctx.fillStyle = '#ffd700'; // Soleil
    } else {
        ctx.fillStyle = '#ffffff'; // Lune
    }
    ctx.fill();
    
    // 2. Saisons (très simple, basé sur l'hémisphère Nord)
    const month = new Date().getMonth();
    let season = '';
    if (month >= 2 && month <= 4) season = 'Printemps';
    else if (month >= 5 && month <= 7) season = 'Été';
    else if (month >= 8 && month <= 10) season = 'Automne';
    else season = 'Hiver';

    // 3. Affichage du temps
    safeSetText('minecraft-time-display', `${time_status} | Saisons : ${season} | ${phase_nom}`);
}

// ========================
// GESTION DU COCKPIT (GÉNÉRAL)
// ========================

function demarrerCockpit() {
  if (!("geolocation" in navigator)) {
    safeSetText('gps', 'GPS non disponible');
    return;
  }
  if (watchId !== null) return;

  tempsDebut = Date.now();
  intervalleTemps = setInterval(afficherTemps, 100);

  // Demarrage GPS
  watchId = navigator.geolocation.watchPosition(
    miseAJourVitesse,
    err => {
      console.warn('GPS refusé ou erreur', err);
      safeSetText('gps', 'Erreur GPS: ' + err.code);
    }, {
      enableHighAccuracy: true,
      maximumAge: 0,
      timeout: 10000
    }
  );
  
  // Demarrage des capteurs
  activerCapteurs();

  document.getElementById('marche').textContent = '▶️ En cours...';
  document.getElementById('marche').classList.remove('pulsation');
  document.getElementById('stop').classList.add('pulsation');
}

function arreterCockpit() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  if (intervalleTemps !== null) clearInterval(intervalleTemps);
  
  watchId = null;
  intervalleTemps = null;
  positionPrecedente = null;

  document.getElementById('marche').textContent = '▶️ Marche';
  document.getElementById('marche').classList.add('pulsation');
  document.getElementById('stop').classList.remove('pulsation');
}

function resetCockpit() {
  arreterCockpit();
  
  vitesseMax = 0;
  vitesses = [];
  distanceTotale = 0;
  tempsDebut = null;

  safeSetText('temps', 'Temps : 0.00 s');
  afficherVitesse(0);
  afficherDistance();
  safeSetText('gps', 'GPS : --');
  afficherGrandeurs(0);
  
  capteursEtat = { niveauBulle: '--', lumiere: '--', son: '--', magnetisme: '--' };
  afficherCapteurs();
}

function toggleModeSouterrain() {
    modeSouterrainActif = !modeSouterrainActif;
    const btn = document.getElementById('toggle-souterrain');
    
    if (modeSouterrainActif) {
        if (watchId !== null) {
            navigator.geolocation.clearWatch(watchId);
            watchId = null;
            positionPrecedente = null; 
        }

        btn.textContent = '✅ Mode Souterrain : ON';
        btn.style.background = '#ff4500';
        btn.style.borderColor = '#ffffff';
        safeDisplay('mode-souterrain-indicator', 'block');
        safeSetText('gps', 'GPS : Signal perdu (Souterrain)');
        
        afficherVitesse(0); 
    } else {
        if (document.getElementById('marche').textContent === '▶️ En cours...') {
            demarrerCockpit(); 
        }

        btn.textContent = '🚫 Mode Souterrain : OFF';
        btn.style.background = '#440000';
        btn.style.borderColor = '#ff4500';
        safeDisplay('mode-souterrain-indicator', 'none');
        safeSetText('gps', 'GPS : Acquisition...');
    }
}

// ========================
// CAPTEURS ET HORLOGE SYSTÈME
// ========================

function afficherCapteurs() {
  safeSetText('capteurs', 
    `Lumière : ${capteursEtat.lumiere} lux | Son : ${capteursEtat.son} dB | Niveau : ${capteursEtat.niveauBulle}° | Gyro : -- | Magnétomètre : ${capteursEtat.magnetisme} µT | Batterie : --% | Réseau : --`
  );
}
async function activerCapteurs(){
  // Activation du listener DeviceOrientation pour la boussole
  if (window.DeviceOrientationEvent && !deviceOrientationListener) {
      deviceOrientationListener = handleOrientation;
      window.addEventListener('deviceorientation', deviceOrientationListener, true);
  }
  
  // Simulacre d'attente pour les autres capteurs
  afficherCapteurs(); 
  capteursEtat.lumiere = 'Attente'; 
  capteursEtat.magnetisme = 'Attente';
  capteursEtat.niveauBulle = 'Attente';
  afficherCapteurs();
}

function activerHorloge(){
  function loop(){
    const t=new Date();
    safeSetText('horloge', `⏰ ${t.getHours().toString().padStart(2,'0')}:${t.getMinutes().toString().padStart(2,'0')}:${t.getSeconds().toString().padStart(2,'0')} (Système)`);
    safeSetText('horloge-atomique', `Heure atomique (UTC) : ${t.getUTCHours().toString().padStart(2,'0')}:${t.getUTCMinutes().toString().padStart(2,'0')}:${t.getUTCSeconds().toString().padStart(2,'0')}`);
    requestAnimationFrame(loop);
  }
  requestAnimationFrame(loop);
}

// ========================
// INIT
// ========================
document.addEventListener('DOMContentLoaded', () => {
 
