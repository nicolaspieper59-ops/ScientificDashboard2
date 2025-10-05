// cockpit.js

// ========================
// ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = []; // Pour calculer la moyenne
let distanceTotale = 0;
let tempsDebut = null; // Pour le calcul du temps total
let intervalleTemps = null; // Pour le suivi de l'horloge système et des capteurs

// Objet pour stocker l'état des capteurs physiques (pour affichage unifié)
let capteursEtat = {
  niveauBulle: '0.0', // Simulé
  lumiere: '700', // Simulé
  son: '55', // Simulé
  magnetisme: '30' // Simulé
};

// ========================
// UTILS BASE TEXT & DOM
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.innerHTML = text; // Utilisation de innerHTML pour les retours à la ligne/balises
}

// ========================
// CALCULS GPS / VITESSE / DISTANCE
// ========================

// Constantes
const VITESSE_LUMIERE = 299792458; // m/s
const VITESSE_SON = 343; // m/s (approximatif)
const R_TERRE = 6371e3; // Rayon de la Terre en mètres
const ANNEE_LUMIERE_SECONDES = 3600 * 24 * 365.25;

function toRad(degrees) {
    return degrees * Math.PI / 180;
}

function calculerDistanceHaversine(p1, p2) {
  const φ1 = toRad(p1.latitude);
  const φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude - p1.latitude);
  const Δλ = toRad(p2.longitude - p1.longitude);
  
  const a = Math.sin(Δφ / 2) ** 2 + 
            Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
            
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R_TERRE * c; // Distance en mètres
}

function calculerVitesseManuelle(gps) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }
  
  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000; // temps en secondes
  const d = calculerDistanceHaversine(gps, positionPrecedente); // distance en mètres

  if (dt <= 0 || d < 0) {
    positionPrecedente = gps;
    return 0;
  }

  // Mise à jour de la distance totale
  distanceTotale += d;
  positionPrecedente = gps;
  
  return (d / dt) * 3.6; // Vitesse en km/h
}

function afficherVitesse(v_kmh) {
  const mps = v_kmh / 3.6;
  const mmps = mps * 1000;
  const moyenne_kmh = vitesses.length ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length : 0;
  
  vitesseMax = Math.max(vitesseMax, v_kmh);

  // Affichage Vitesse
  safeSetText('vitesse', `Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);

  // Affichage Pourcentage
  safeSetText('pourcentage', `% Lumière : ${(mps / VITESSE_LUMIERE * 100).toExponential(2)}% | % Son : ${(mps / VITESSE_SON * 100).toFixed(2)}%`);
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  const m = distanceTotale;
  const secLumiere = m / VITESSE_LUMIERE;
  const al = secLumiere / (ANNEE_LUMIERE_SECONDES * VITESSE_LUMIERE / 1000); // Conversion d'unités pour être précis (m/s)

  // Affichage Distance
  safeSetText('distance', `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${Math.round(m * 1000)} mm`);
  safeSetText('distance-cosmique', `Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
  if (!tempsDebut) return;
  const tempsEcoule = (Date.now() - tempsDebut) / 1000;
  safeSetText('temps', `Temps : ${tempsEcoule.toFixed(2)} s`);
}


function miseAJourVitesse(pos) {
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: pos.timestamp,
    speed: pos.coords.speed // Vitesse native en m/s
  };

  let vitesse_kmh;

  if (typeof gps.speed === 'number' && gps.speed >= 0) {
      // 1. Vitesse Native GPS (plus fiable)
      vitesse_kmh = gps.speed * 3.6;
      // Il faut quand même calculer la distance manuellement
      if (positionPrecedente) {
          distanceTotale += calculerDistanceHaversine(gps, positionPrecedente);
      }
      positionPrecedente = gps;
  } else {
      // 2. Vitesse calculée (Haversine)
      vitesse_kmh = calculerVitesseManuelle(gps); // Met à jour distanceTotale et positionPrecedente
  }
  
  if (vitesse_kmh >= 0) {
    vitesses.push(vitesse_kmh);
    if (vitesses.length > 60) vitesses.shift();

    afficherVitesse(vitesse_kmh);
    afficherDistance();
    afficherTemps();
  }

  // Affichage GPS
  safeSetText('gps', `GPS : Latitude : ${gps.latitude.toFixed(6)} | Longitude : ${gps.longitude.toFixed(6)} | Altitude : ${gps.altitude?.toFixed(0)??'--'} m | Précision GPS : ${gps.accuracy?.toFixed(0)??'--'} m`);
  
  // Affichage Boussole (simulation si pas de capteur)
  safeSetText('compass-display', 'Boussole (Nord Vrai) : 0.0° (Simulé)');
}

// ========================
// GESTION DU COCKPIT
// ========================

function demarrerCockpit() {
  if (!("geolocation" in navigator)) {
    safeSetText('gps', 'GPS non disponible');
    return;
  }
  
  // ** VÉRIFICATION HTTPS CRITIQUE **
  if (window.location.protocol !== 'https:') {
    alert("⚠️ La géolocalisation et les capteurs nécessitent une connexion sécurisée (HTTPS).");
    safeSetText('gps', 'GPS : **HTTPS REQUIS**');
    return;
  }

  if (watchId !== null) {
      arreterCockpit(); // Toggle Marche/Arrêt
      return;
  }

  tempsDebut = Date.now();
  afficherTemps();
  
  // Démarrage du suivi GPS
  watchId = navigator.geolocation.watchPosition(
    miseAJourVitesse,
    err => {
      console.warn('GPS refusé ou erreur', err);
      safeSetText('gps', `Erreur GPS: Code ${err.code} | Message : ${err.message}`);
    }, {
      enableHighAccuracy: true,
      maximumAge: 0,
      timeout: 10000
    }
  );
  
  // Démarrage du suivi temps/horloge/capteurs
  if (!intervalleTemps) {
      intervalleTemps = setInterval(() => {
          afficherTemps();
          updateSystemClock();
          afficherCapteurs();
          // updateSolaire est appelé dans updateSystemClock
      }, 1000);
      updateSystemClock(); // Premier appel immédiat
  }
  
  document.getElementById('marche').textContent = '⏹️ Arrêt';
}

function arreterCockpit() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  
  if (intervalleTemps !== null) clearInterval(intervalleTemps);
  intervalleTemps = null;
  
  // Réinitialisation de l'état temporaire
  positionPrecedente = null;
  vitesses = [];
  tempsDebut = null; 
  
  safeSetText('vitesse', 'Vitesse instantanée : **ARRÊT**');
  document.getElementById('marche').textContent = '▶️ Marche';
}

function resetCockpit() {
    arreterCockpit(); // Arrête tout
    
    // Réinitialisation totale des données
    vitesseMax = 0;
    distanceTotale = 0;
    
    // Réinitialisation des affichages
    safeSetText('temps', 'Temps : 0.00 s');
    afficherVitesse(0);
    afficherDistance();
    safeSetText('gps', 'GPS : --');
    safeSetText('compass-display', 'Boussole (Nord Vrai) : --°');
}

// ========================
// CAPTEURS ET HORLOGE SYSTÈME/MINECRAFT
// ========================

function updateSystemClock() {
    const now = new Date();
    safeSetText('horloge', `⏰ ${now.toLocaleTimeString('fr-FR')} (Système)`);
    updateSolaire(now);
}

function updateSolaire(now) {
    const secondsInDay = now.getHours() * 3600 + now.getMinutes() * 60 + now.getSeconds();
    
    // Simulation de l'heure Minecraft (24h en 20 minutes réelles)
    const minecraftDayLengthSeconds = 20 * 60; 
    const realTimeSinceMidnight = (secondsInDay % minecraftDayLengthSeconds);
    const timeOfDay = realTimeSinceMidnight / minecraftDayLengthSeconds; // 0.0 à 1.0
    
    // Affichage des heures solaires (simulées)
    safeSetText('heure-vraie', `Heure Solaire Vraie : ${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}`);
    safeSetText('lune-phase', `Phase Lune : ${timeOfDay < 0.5 ? 'Croissant' : 'Décroissant'} (Sim.)`);
    
    // Dessin de l'horloge Minecraft
    const canvas = document.getElementById('minecraft-clock');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const radius = 45;

    ctx.fillStyle = '#111';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = '#00ffcc';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.stroke();

    let phaseText = 'Jour';
    let color = '#ffd700';
    if (timeOfDay > 0.8 || timeOfDay < 0.2) {
        phaseText = 'Nuit';
        color = '#00ffcc';
    }
    
    let angle = (timeOfDay * 2 * Math.PI) - (Math.PI / 2); // 0.5 = midi, 0.0/1.0 = minuit

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(cx + radius * Math.cos(angle), cy + radius * Math.sin(angle), 5, 0, 2 * Math.PI);
    ctx.fill();

    safeSetText('minecraft-time-display', `Jour/Nuit : ${phaseText} (${(timeOfDay * 24).toFixed(1)}h M-Craft)`);
}

function afficherCapteurs() {
  let bat_level = Math.floor(Math.random() * 50) + 50; 
  
  if ('getBattery' in navigator) {
      navigator.getBattery().then(battery => {
          bat_level = Math.floor(battery.level * 100);
          safeSetText('capteurs', 
            `Lumière : ${capteursEtat.lumiere} lux | Son : ${capteursEtat.son} dB | Niveau : ${capteursEtat.niveauBulle}° | Magnétomètre : ${capteursEtat.magnetisme} µT | Batterie : ${bat_level}% | Réseau : 5G`
          );
      });
  } else {
      safeSetText('capteurs', 
        `Lumière : ${capteursEtat.lumiere} lux | Son : ${capteursEtat.son} dB | Niveau : ${capteursEtat.niveauBulle}° | Magnétomètre : ${capteursEtat.magnetisme} µT | Batterie : ${bat_level}% | Réseau : 5G`
      );
  }
}

// ========================
// INITIALISATION
// ========================

document.addEventListener('DOMContentLoaded', () => {
  // 1. Initialisation des affichages
  afficherVitesse(0);
  afficherDistance();
  safeSetText('temps', 'Temps : 0.00 s');
  afficherCapteurs();
  updateSystemClock();

  // 2. Mise en place des événements click
  // Marche devient Marche/Arrêt
  document.getElementById('marche').addEventListener('click', demarrerCockpit); 
  // Arrêt est masqué par défaut, peut servir pour une double sécurité
  document.getElementById('stop').addEventListener('click', arreterCockpit); 
  document.getElementById('reset').addEventListener('click', () => {
      vitesseMax = 0;
      safeSetText('vitesse-max', 'Vitesse max : 0.00 km/h');
  });
  document.getElementById('reset-tout').addEventListener('click', resetCockpit);
  
  // 3. Gestion de l'orientation (Boussole) - nécessite HTTPS et parfois permission explicite
  if (window.DeviceOrientationEvent) {
      // Pour les appareils mobiles, une demande de permission pourrait être nécessaire ici
      // mais on ajoute l'écouteur directement pour les navigateurs de bureau ou Android non-restreints.
      window.addEventListener('deviceorientation', (event) => {
          let alpha = event.webkitCompassHeading || event.alpha;
          if (alpha !== null && alpha !== undefined) {
              let heading = (alpha > 0) ? 360 - alpha : -alpha;
              safeSetText('compass-display', `Boussole (Nord Vrai) : ${heading.toFixed(2)}°`);
          }
      }, true);
  } else {
      safeSetText('compass-display', 'Boussole (Nord Vrai) : **NON DISPONIBLE**');
  }
});
