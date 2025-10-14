// ========================
// 1. État global & Constantes
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;
let modeSouterrainActif = false;
let targetCoords = { latitude: 48.8584, longitude: 2.2945 };

// Constantes physiques
const VITESSE_LUMIERE = 299792458;
const VITESSE_SON = 343;
const R_TERRE = 6371e3; 
const ANNEE_LUMIERE_SECONDES = 3600*24*365.25;

// ========================
// 2. Utilitaires
// ========================
function safeSetText(id, text) { 
    const e = document.getElementById(id); 
    // Assure qu'on écrit dans le span si l'ID est sur le conteneur <p>
    const target = e?.querySelector('span') || e;
    if(target) target.textContent = text;
}
function toRadians(d){return d*Math.PI/180;}
function toDegrees(r){return r*180/Math.PI;}

// Calculs Haversine (distance) et Relèvement (angle vers la cible)
function calculerDistanceHaversine(p1,p2){ /* ... */ return 0;} 
function calculerRelèvement(p1, p2) { /* ... */ return 0;}


// ========================
// 3. Calculs Astronomiques (Simplifiés)
// Basé sur des algorithmes standard pour l'Équation du Temps (EoT)
// ========================

// Convertit une Date JavaScript en Jour Julien (utilisé en astronomie)
function dateToJD(date) {
    // Calcul simplifié pour l'astronomie de la position (suffisant pour EoT)
    const msSinceEpoch = date.getTime();
    return 2440587.5 + (msSinceEpoch / 86400000); 
}

// Calcul de l'Équation du Temps (EoT) et de la Longitude Solaire
function calculerHeuresSolaires(date, latitude, longitude) {
    const jd = dateToJD(date);
    const n = jd - 2451545.0; // Jours depuis J2000.0

    // Anomalie moyenne du Soleil (M)
    const M_deg = 357.5291 + 0.98560028 * n;
    const M = toRadians(M_deg % 360);

    // Longitude écliptique du Soleil (L)
    const L_deg = 280.459 + 0.98564736 * n;
    const L = toRadians(L_deg % 360);

    // Composante de l'excentricité (C) - Correction du centre
    const C = toDegrees(1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M));
    
    // Longitude Solaire (L_s)
    const L_s = (L_deg + C) % 360;

    // Angle Solaire Vrai (RA) - Équivalent de l'ascension droite
    const RA = toDegrees(Math.atan2(
        Math.cos(toRadians(23.44)) * Math.sin(toRadians(L_s)), 
        Math.cos(toRadians(L_s))
    ));
    
    // Équation du Temps (EoT) en minutes
    // EoT = 4 * (Heure Solaire Moyenne - Heure Solaire Vraie)
    const EoT_min = (L_deg % 360) / 15 - RA / 15;
    
    // Heure Solaire Moyenne (HSM) - Heure locale corrigée par la longitude
    const heureUTC = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const t_HSM = heureUTC + longitude / 15;
    const HSM = (t_HSM % 24 + 24) % 24;
    
    // Heure Solaire Vraie (HSV)
    const HSV = (HSM + EoT_min / 60) % 24;

    // Affichage des données (Simplifié)
    safeSetText('date-display', date.toLocaleString());
    safeSetText('hsv', formatHeure(HSV));
    safeSetText('hsm', formatHeure(HSM));
    safeSetText('eq-temps', `${(EoT_min * 60).toFixed(1)} s`);
    safeSetText('solar-lon', `${L_s.toFixed(2)}°`);
    safeSetText('solar-day-duration', `24h 00min 00s`); // Simplifié pour le moment

    return { EoT_min, L_s };
}

// Formatage de l'heure flottante en HH:MM:SS
function formatHeure(heureFloat) {
    const totalSecs = heureFloat * 3600;
    const h = Math.floor(totalSecs / 3600);
    const m = Math.floor((totalSecs % 3600) / 60);
    const s = Math.floor(totalSecs % 60);
    return `${String(h).padStart(2, '0')}h ${String(m).padStart(2, '0')}min ${String(s).padStart(2, '0')}s`;
}


// ========================
// 4. Fonctions GPS & Cinématique
// ========================
function miseAJourVitesse(pos) {
    const now = Date.now();
    if (!tempsDebut) tempsDebut = now;

    // ... (Logique Mode Souterrain, Filtre de Précision) ...

    const gps = {
        latitude: pos.coords.latitude,
        longitude: pos.coords.longitude,
        altitude: pos.coords.altitude || 0,
        accuracy: pos.coords.accuracy,
        speed: pos.coords.speed || 0,
        heading: pos.coords.heading || 0
    };

    let vitesse_ms = gps.speed; 
    
    if (positionPrecedente) {
        // ... (Calculs de Vitesse/Distance si gps.speed n'est pas fiable) ...
    }
    
    positionPrecedente = gps;

    const vitesse_kmh = vitesse_ms * 3.6;
    vitesseMax = Math.max(vitesseMax, vitesse_kmh);
    vitesses.push(vitesse_kmh); if (vitesses.length > 30) vitesses.shift();

    // MISE À JOUR ASTRONOMIE
    calculerHeuresSolaires(new Date(), gps.latitude, gps.longitude);

    // MISE À JOUR CAPTEURS NATIFS SIMULÉS
    simulerCapteursNatifs(vitesse_kmh); 
    
    // Affichage Cinématique
    afficherVitesse(vitesse_kmh, vitesse_ms);
    afficherDistance();
    afficherTemps();
    afficherGPS(gps);
    // ... (Mettre à jour la carte et la cible)
}

function afficherVitesse(v_kmh, v_ms) {
    const moyenne_kmh = vitesses.reduce((a, b) => a + b, 0) / vitesses.length || 0;
    
    safeSetText('vitesse', `${v_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-moy', `${moyenne_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-max', `${vitesseMax.toFixed(2)} km/h`);
    safeSetText('vitesse-ms', `${v_ms.toFixed(2)} m/s | ${(v_ms * 1000).toFixed(0)} mm/s`);
    safeSetText('pourcentage', `${(v_ms / VITESSE_LUMIERE * 100).toExponential(2)}% | ${(v_ms / VITESSE_SON * 100).toFixed(2)}%`);
}

function afficherDistance() {
    const km = distanceTotale / 1000, m = distanceTotale, mm = m * 1000;
    const secLumiere = m / VITESSE_LUMIERE, al = secLumiere / ANNEE_LUMIERE_SECONDES;
    safeSetText('distance', `${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm`);
    safeSetText('distance-cosmique', `${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
    if (!tempsDebut) return;
    const t = (Date.now() - tempsDebut) / 1000;
    safeSetText('temps', `${t.toFixed(2)} s`);
}

function afficherGPS(g) {
    safeSetText('gps', `Lat : ${g.latitude.toFixed(4)} | Lon : ${g.longitude.toFixed(4)} | Alt : ${g.altitude.toFixed(0)} m | Préc. : ${g.accuracy.toFixed(0)} m`);
    safeSetText('compass-display', `${g.heading.toFixed(1)}°`);
}


// ========================
// 5. Simulation des Capteurs (Natif)
// Ces fonctions nécessitent une API native (Capacitor/Cordova) pour être réelles.
// ========================
let batterie = 85; 

function simulerCapteursNatifs(v_kmh) {
    // Lumière : Simule en fonction de l'heure du jour (basé sur un simple sinusoïde)
    const now = new Date();
    const h = now.getHours() + now.getMinutes() / 60;
    const lux = 100 + 40000 * Math.sin(Math.PI * (h - 6) / 12) + Math.random() * 50; 
    safeSetText('cap-lumiere', `${lux < 0 ? 0 : lux.toFixed(0)} lux`);

    // Pression Est. : Simplifiée, juste une constante
    safeSetText('pression-est', `1013.25 hPa`);

    // Énergie Cinétique Est. : E = 0.5 * m * v^2 (Simule une masse de 80 kg)
    const energie = 0.5 * 80 * (v_kmh / 3.6)**2;
    safeSetText('energie-cinetique', `${energie.toFixed(0)} J`);

    // Batterie : Simule une décharge lente
    batterie = Math.max(0, batterie - 0.0001);
    safeSetText('cap-batterie', `${batterie.toFixed(1)}%`);

    // Affichage des autres placeholders pour compléter l'interface
    safeSetText('cap-son', `-- dB`);
    safeSetText('cap-niveau', `--°`);
    safeSetText('cap-gyro', `--`);
    safeSetText('cap-mag', `--`);
    safeSetText('cap-reseau', `4G (Simulé)`);
    safeSetText('heure-atomique', `${now.toUTCString().split(' ')[4]} UTC`);
    safeSetText('jour-nuit', (now.getHours() > 6 && now.getHours() < 20) ? 'Jour ☀️' : 'Nuit 🌙');
    
    // Placeholder Lunaire (Calculs complexes non inclus)
    safeSetText('phase-lune', `Nouvelle Lune (Simulé)`);
    safeSetText('culmination-lune', `-- (Simulé)`);
    safeSetText('mag-lune', `-12.7 (Simulé)`);
    safeSetText('lever-soleil', `06h 00min`);
    safeSetText('coucher-soleil', `18h 00min`);
}


// ========================
// 6. Contrôles & Initialisation
// ========================
function demarrerCockpit(){
  if(watchId!==null) return;
  if(!tempsDebut) tempsDebut=Date.now();
  const options={enableHighAccuracy:true,timeout:5000,maximumAge:0};
  
  watchId=navigator.geolocation.watchPosition(
    miseAJourVitesse,
    (e)=>{console.error(e);safeSetText('gps',`Erreur GPS (${e.code})`)},
    options
  );

  document.getElementById('marche').textContent = '⏸️ Pause';
}

function arreterCockpit(){
  if(watchId!==null){navigator.geolocation.clearWatch(watchId); watchId=null;}
  document.getElementById('marche').textContent = '▶️ Marche';
  safeSetText('vitesse','PAUSE');
}

function resetCockpit(){
    arreterCockpit(); positionPrecedente=null; vitesseMax=0; vitesses=[]; distanceTotale=0; tempsDebut=null;
    
    // Reset affichages
    const initialText = ' --';
    safeSetText('vitesse', initialText + ' km/h'); 
    safeSetText('vitesse-moy', initialText + ' km/h'); 
    safeSetText('vitesse-max', initialText + ' km/h'); 
    safeSetText('vitesse-ms', initialText + ' m/s |' + initialText + ' mm/s'); 
    safeSetText('pourcentage', initialText + '% |' + initialText + '%');
    safeSetText('distance', initialText + ' km |' + initialText + ' m |' + initialText + ' mm');
    safeSetText('distance-cosmique', initialText + ' s lumière |' + initialText + ' al');
    safeSetText('temps', initialText + ' s');
    safeSetText('gps', 'En attente...');
    
    // Reset Astro/Capteurs
    safeSetText('eq-temps', initialText);
    safeSetText('solar-lon', initialText);
    safeSetText('hsv', initialText);
    safeSetText('hsm', initialText);
    safeSetText('date-display', new Date().toLocaleDateString());
    safeSetText('pression-est', initialText + ' hPa');
    // ... (Réinitialiser tous les autres champs si nécessaire)

    // Logique de carte non incluse.
}

// Initialisation des écouteurs d'événements
document.addEventListener('DOMContentLoaded', () => {
    document.getElementById('marche').addEventListener('click', demarrerCockpit);
    document.getElementById('arreter').addEventListener('click', arreterCockpit);
    document.getElementById('reset').addEventListener('click', resetCockpit);
    document.getElementById('toggle-souterrain').addEventListener('click', () => {
        modeSouterrainActif = !modeSouterrainActif;
        document.getElementById('toggle-souterrain').textContent = `🚫 Mode Souterrain : ${modeSouterrainActif ? 'ON' : 'OFF'}`;
        if (modeSouterrainActif) arreterCockpit();
    });

    // Autres écouteurs pour la cible, etc.
    document.getElementById('target-coord').value = `${targetCoords.latitude}, ${targetCoords.longitude}`;
    
    // Initialisation
    resetCockpit();
    simulerCapteursNatifs(0); // Afficher les valeurs simulées initiales
});
