// ========================
// État global
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;
let modeSouterrainActif = false;
let targetCoords = { latitude: 48.8584, longitude: 2.2945 };
let modulesActive = { rituel:true, physique:true, chimie:true, svt:true };
let map; // Variable pour la carte Leaflet
let marker; // Marqueur de position actuelle

// Variables Barométriques (Simulées)
let pressionAtmospherique = 1013.25; // Pression par défaut en hPa
let vitesseVerticaleBaro = 0;

// Constantes physiques
const VITESSE_LUMIERE = 299792458;
const VITESSE_SON = 343;
const R_TERRE = 6371e3;
const ANNEE_LUMIERE_SECONDES = 3600*24*365.25;
const MASSE_EXPLORATEUR = 70;

// Constantes Barométriques
const P0 = 1013.25; // Pression au niveau de la mer standard (hPa)

// ========================
// Utilitaires
// ========================
function safeSetText(id,text){ const e=document.getElementById(id); if(e) e.textContent=text; }
function toRadians(d){return d*Math.PI/180;}
function toDegrees(r){return r*180/Math.PI;}

function calculerDistanceHaversine(p1,p2){
  const φ1=toRadians(p1.latitude), φ2=toRadians(p2.latitude);
  const Δφ=toRadians(p2.latitude-p1.latitude), Δλ=toRadians(p2.longitude-p1.longitude);
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return R_TERRE*2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
}

function calculerRelèvement(p1, p2) {
    const λ1 = toRadians(p1.longitude);
    const λ2 = toRadians(p2.longitude);
    const φ1 = toRadians(p1.latitude);
    const φ2 = toRadians(p2.latitude);

    const y = Math.sin(λ2 - λ1) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(λ2 - λ1);
    let brng = Math.atan2(y, x);

    brng = toDegrees(brng);
    return (brng + 360) % 360;
}

// Calcul approximatif de l'altitude à partir de la pression (formule simplifiée de l'atmosphère standard)
function calculerAltitudeBarometrique(pressionHpa, pBase = P0) {
  // Altitude en mètres
  return 44330 * (1 - Math.pow(pressionHpa / pBase, 1 / 5.255));
}


// ========================
// GPS & vitesse
// ========================
function miseAJourVitesse(pos){
  const now = Date.now();
  if(!tempsDebut) tempsDebut = now;

  // Filtre mode souterrain (Affiche les 0.00 et sort)
  if(modeSouterrainActif) {
    resetDisplayToZero();
    safeSetText('gps',`GPS : Mode Souterrain - Données Figées`);
    return;
  }
  
  // Filtre de précision (Ignorer si la précision est trop mauvaise, par exemple > 25 mètres)
  if(pos.coords.accuracy > 25) {
    safeSetText('gps', `GPS : Signal faible (${pos.coords.accuracy.toFixed(0)} m). Attente de précision...`);
    // On peut choisir d'ignorer tout le calcul ou de ne mettre à jour que le GPS
    return; 
  }

  const gps={
    latitude:pos.coords.latitude,
    longitude:pos.coords.longitude,
    altitude:pos.coords.altitude||0,
    accuracy:pos.coords.accuracy,
    speed:pos.coords.speed||0,
    heading:pos.coords.heading // Direction du déplacement
  };
  
  // NOUVEAU : SIMULATION DE LA PRESSION
  // 1. Calculer la pression correspondant à l'altitude GPS pour avoir une base réaliste
  const PRESSION_BASE_GPS = P0 / Math.pow(1 - (gps.altitude / 44330), 5.255); 
  // 2. Ajouter une petite variation pour simuler le bruit du capteur ou le temps
  pressionAtmospherique = PRESSION_BASE_GPS + (Math.sin(now / 100000) * 1); 

  let vitesse_ms=0;
  let vitesse_verticale_gps = 0;
  
  if(positionPrecedente){
    const dt=(pos.timestamp-positionPrecedente.timestamp)/1000;
    const dist=calculerDistanceHaversine(gps,positionPrecedente);
    distanceTotale+=dist;
    
    // Calcul de la Vitesse Horizontale
    vitesse_ms=(gps.speed>0 && gps.speed<VITESSE_LUMIERE) ? gps.speed : (dt > 0 ? dist/dt : 0);
    
    // Calcul de la Vitesse Verticale GPS
    const deltaAltitudeGPS = gps.altitude - positionPrecedente.altitude;
    vitesse_verticale_gps = (dt > 0) ? deltaAltitudeGPS / dt : 0;
    
    // Calcul de la Vitesse Verticale Barométrique
    const altitudePrecedenteBaro = calculerAltitudeBarometrique(positionPrecedente.pression || P0);
    const altitudeActuelleBaro = calculerAltitudeBarometrique(pressionAtmospherique);

    const deltaAltitudeBaro = altitudeActuelleBaro - altitudePrecedenteBaro;
    vitesseVerticaleBaro = (dt > 0) ? deltaAltitudeBaro / dt : 0;

  } else {
    // Initialisation
    calculerHeuresSolaires(new Date(), gps.latitude, gps.longitude);
    initialiserCarte(gps);
  }
  
  // Stocker la position et la pression pour le prochain calcul
  positionPrecedente = {...gps, pression: pressionAtmospherique}; 

  const vitesse_kmh=vitesse_ms*3.6;
  vitesseMax=Math.max(vitesseMax,vitesse_kmh);
  vitesses.push(vitesse_kmh); if(vitesses.length>30) vitesses.shift();
  
  // Affichage
  afficherVitesse(vitesse_kmh, vitesse_ms, vitesse_verticale_gps, vitesseVerticaleBaro); 
  afficherDistance();
  afficherTemps();
  afficherGPS(gps);
  mettreAJourCarte(gps);
  calculerCible(gps);
}

function afficherVitesse(v_kmh,v_ms, v_verticale_gps, v_verticale_baro){
  const moyenne_kmh=vitesses.reduce((a,b)=>a+b,0)/vitesses.length||0;
  
  // Conversion de m/s en m/min (lecture intuitive)
  const v_vert_gps_m_min = v_verticale_gps * 60;
  const v_vert_baro_m_min = v_verticale_baro * 60;
  const direction_baro = v_verticale_baro > 0 ? '↗️ Montée' : (v_verticale_baro < 0 ? '↘️ Descente' : '↔️ Stable');

  safeSetText('vitesse',`Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy',`Vitesse moyenne (30s) : ${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms',`Vitesse Horizontale : ${v_ms.toFixed(2)} m/s | ${(v_ms*1000).toFixed(0)} mm/s`);
  
  // Affichage Barométrique
  safeSetText('vitesse-verticale-baro',`Vitesse Verticale Baro : ${v_vert_baro_m_min.toFixed(1)} m/min (${v_verticale_baro.toFixed(2)} m/s) ${direction_baro}`);
  safeSetText('pression-air', `Pression : ${pressionAtmospherique.toFixed(2)} hPa | Alt Baro: ${calculerAltitudeBarometrique(pressionAtmospherique).toFixed(0)} m`);
  
  // Affichage GPS (pour comparaison)
  safeSetText('vitesse-verticale-gps', `Vitesse Verticale GPS : ${v_vert_gps_m_min.toFixed(1)} m/min (${v_verticale_gps.toFixed(2)} m/s)`);

  safeSetText('pourcentage',`% Lumière : ${(v_ms/VITESSE_LUMIERE*100).toExponential(2)}% | % Son : ${(v_ms/VITESSE_SON*100).toFixed(2)}%`);
}

function afficherDistance(){
  const km=distanceTotale/1000, m=distanceTotale, mm=m*1000;
  const secLumiere=m/VITESSE_LUMIERE, al=secLumiere/ANNEE_LUMIERE_SECONDES;
  safeSetText('distance',`Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm`);
  safeSetText('distance-cosmique',`Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps(){
  if(!tempsDebut) return;
  const t=(Date.now()-tempsDebut)/1000;
  safeSetText('temps',`Temps de mission : ${t.toFixed(2)} s`);
}

function afficherGPS(g){
  safeSetText('gps',`GPS : Lat : ${g.latitude.toFixed(6)} | Lon : ${g.longitude.toFixed(6)} | Alt : ${g.altitude.toFixed(0)} m | Préc. : ${g.accuracy.toFixed(0)} m`);
  safeSetText('compass-display',`Boussole (Cap GPS) : ${g.heading ? g.heading.toFixed(1) + '°' : 'En attente...'}`);
}

// ... (calculerHeuresSolaires, drawMinecraftClock) - Omitted for brevity, assuming they are defined but simplified

// ========================
// Carte Cosmique & Cible
// ========================

// ... (initialiserCarte, mettreAJourCarte) - Omitted for brevity

function calculerCible(pos) {
    const bearing = calculerRelèvement(pos, targetCoords);
    const distance_m = calculerDistanceHaversine(pos, targetCoords);
    const distance_km = distance_m / 1000;

    safeSetText('bearing-display', `Relèvement vers la cible : ${bearing.toFixed(1)}° | Distance : ${distance_km.toFixed(3)} km`);
    // Note: Leaflet markers are usually handled in mettreAJourCarte or initialiserCarte for simplicity
}

// ========================
// Contrôles cockpit
// ========================
function demarrerCockpit(){
  if(watchId!==null) return;
  if(!tempsDebut) tempsDebut=Date.now();
  const options={enableHighAccuracy:true,timeout:5000,maximumAge:0};
  watchId=navigator.geolocation.watchPosition(miseAJourVitesse,(e)=>{console.error(e);safeSetText('gps',`GPS : Erreur (${e.code})`)},options);
}

function arreterCockpit(){
  if(watchId!==null){navigator.geolocation.clearWatch(watchId); watchId=null;}
  safeSetText('vitesse','Vitesse instantanée : **PAUSE**');
  document.getElementById('marche').textContent = '▶️ Marche';
}

function resetDisplayToZero(){
  safeSetText('vitesse','Vitesse instantanée : 0.00 km/h'); 
  safeSetText('vitesse-moy','Vitesse moyenne : 0.00 km/h'); 
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
  safeSetText('vitesse-ms','Vitesse Horizontale : 0.00 m/s | 0 mm/s'); 
  safeSetText('vitesse-verticale-baro','Vitesse Verticale Baro : 0.0 m/min (0.00 m/s) ↔️ Stable'); 
  safeSetText('vitesse-verticale-gps', 'Vitesse Verticale GPS : 0.0 m/min (0.00 m/s)');
  safeSetText('pression-air', 'Pression : 1013.25 hPa | Alt Baro: 0 m');
  safeSetText('pourcentage','% Lumière : 0.00% | % Son : 0.00%');
  safeSetText('distance','Distance : 0.000 km | 0 m | 0 mm'); 
  safeSetText('distance-cosmique','Distance cosmique : 0.000 s lumière | 0.000e+00 al');
  safeSetText('temps','Temps : 0.00 s'); 
}

function resetCockpit(){
  arreterCockpit(); positionPrecedente=null; vitesseMax=0; vitesses=[]; distanceTotale=0; tempsDebut=null;
  pressionAtmospherique = 1013.25; vitesseVerticaleBaro = 0; // Réinitialisation Barométrique

  resetDisplayToZero();
  safeSetText('gps','GPS : --');
  safeSetText('compass-display','Boussole (Cap GPS) : --°');
  safeSetText('bearing-display', 'Relèvement vers la cible : --° | Distance : -- km');

  if(marker) map.removeLayer(marker);
  map = null; // Réinitialiser la carte pour la prochaine initialisation
  // ... (Assurez-vous d'appeler drawMinecraftClock ou de le définir quelque part)
}

// ... (setTargetCoords, Event Listeners) - Omitted for brevity
