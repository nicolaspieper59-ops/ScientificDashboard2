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

// Variables Barométriques (Simulées)
let pressionAtmospherique = 1013.25; 
let vitesseVerticaleBaro = 0;

// Constantes physiques
const VITESSE_LUMIERE = 299792458;
const VITESSE_SON = 343;
const R_TERRE = 6371e3;
const ANNEE_LUMIERE_SECONDES = 3600*24*365.25;

// Constantes Barométriques
const P0 = 1013.25; 

// Variables de Carte (Laissées nulles pour éviter les erreurs si la logique Leaflet n'est pas complète)
let map = null; 
let marker = null; 

// ========================
// Utilitaires
// ========================
function safeSetText(id,text){ 
    const e=document.getElementById(id); 
    if(e) e.textContent=text;
}
function toRadians(d){return d*Math.PI/180;}

function calculerDistanceHaversine(p1,p2){
  const φ1=toRadians(p1.latitude), φ2=toRadians(p2.latitude);
  const Δφ=toRadians(p2.latitude-p1.latitude), Δλ=toRadians(p2.longitude-p1.longitude);
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return R_TERRE*2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
}

function calculerAltitudeBarometrique(pressionHpa, pBase = P0) {
  return 44330 * (1 - Math.pow(pressionHpa / pBase, 1 / 5.255));
}

function toDegrees(r){return r*180/Math.PI;}

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


// ========================
// GPS & vitesse
// ========================
function miseAJourVitesse(pos){
  const now = Date.now();
  if(!tempsDebut) tempsDebut = now;

  if(modeSouterrainActif) {
    resetDisplayToZero();
    safeSetText('gps',`GPS : Mode Souterrain - Données Figées`);
    return;
  }
  
  if(pos.coords.accuracy > 25) {
    safeSetText('gps', `GPS : Signal faible (${pos.coords.accuracy.toFixed(0)} m). Attente de précision...`);
    return; 
  }

  const gps={
    latitude:pos.coords.latitude,
    longitude:pos.coords.longitude,
    altitude:pos.coords.altitude||0,
    accuracy:pos.coords.accuracy,
    speed:pos.coords.speed||0,
    heading:pos.coords.heading||0 
  };
  
  // SIMULATION DE LA PRESSION
  const PRESSION_BASE_GPS = P0 / Math.pow(1 - (gps.altitude / 44330), 5.255); 
  pressionAtmospherique = PRESSION_BASE_GPS + (Math.sin(now / 100000) * 1); 

  let vitesse_ms=0;
  let vitesse_verticale_gps = 0;
  
  if(positionPrecedente){
    const dt=(pos.timestamp-positionPrecedente.timestamp)/1000;
    const dist=calculerDistanceHaversine(gps,positionPrecedente);
    distanceTotale+=dist;
    
    vitesse_ms=(gps.speed>0 && gps.speed<VITESSE_LUMIERE) ? gps.speed : (dt > 0 ? dist/dt : 0);
    
    const deltaAltitudeGPS = gps.altitude - positionPrecedente.altitude;
    vitesse_verticale_gps = (dt > 0) ? deltaAltitudeGPS / dt : 0;
    
    // Calcul de la Vitesse Verticale Barométrique
    const altitudePrecedenteBaro = calculerAltitudeBarometrique(positionPrecedente.pression || P0);
    const altitudeActuelleBaro = calculerAltitudeBarometrique(pressionAtmospherique);

    const deltaAltitudeBaro = altitudeActuelleBaro - altitudePrecedenteBaro;
    vitesseVerticaleBaro = (dt > 0) ? deltaAltitudeBaro / dt : 0;

  } else {
    // initialiserCarte(gps); // Commenté
  }
  
  positionPrecedente = {...gps, pression: pressionAtmospherique}; 

  const vitesse_kmh=vitesse_ms*3.6;
  vitesseMax=Math.max(vitesseMax,vitesse_kmh);
  vitesses.push(vitesse_kmh); if(vitesses.length>30) vitesses.shift();
  
  // Affichage
  afficherVitesse(vitesse_kmh, vitesse_ms, vitesse_verticale_gps, vitesseVerticaleBaro); 
  afficherDistance();
  afficherTemps();
  afficherGPS(gps);
  // mettreAJourCarte(gps); // Commenté
  calculerCible(gps); 
}

function afficherVitesse(v_kmh,v_ms, v_verticale_gps, v_verticale_baro){
  const moyenne_kmh=vitesses.reduce((a,b)=>a+b,0)/vitesses.length||0;
  
  const v_vert_gps_m_min = v_verticale_gps * 60;
  const v_vert_baro_m_min = v_verticale_baro * 60;
  const direction_baro = v_verticale_baro > 0 ? '↗️ Montée' : (v_verticale_baro < 0 ? '↘️ Descente' : '↔️ Stable');

  safeSetText('vitesse',`${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy',`${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max',`${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms',`${v_ms.toFixed(2)} m/s | ${(v_ms*1000).toFixed(0)} mm/s`);
  
  safeSetText('vitesse-verticale-baro',`${v_vert_baro_m_min.toFixed(1)} m/min (${v_verticale_baro.toFixed(2)} m/s) ${direction_baro}`);
  safeSetText('pression-air', `${pressionAtmospherique.toFixed(2)} hPa | Alt Baro: ${calculerAltitudeBarometrique(pressionAtmospherique).toFixed(0)} m`);
  safeSetText('vitesse-verticale-gps', `${v_vert_gps_m_min.toFixed(1)} m/min (${v_verticale_gps.toFixed(2)} m/s)`);

  safeSetText('pourcentage',`${(v_ms/VITESSE_LUMIERE*100).toExponential(2)}% | ${(v_ms/VITESSE_SON*100).toFixed(2)}%`);
}

function afficherDistance(){
  const km=distanceTotale/1000, m=distanceTotale, mm=m*1000;
  const secLumiere=m/VITESSE_LUMIERE, al=secLumiere/ANNEE_LUMIERE_SECONDES;
  safeSetText('distance',`${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm`);
  safeSetText('distance-cosmique',`${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps(){
  if(!tempsDebut) return;
  const t=(Date.now()-tempsDebut)/1000;
  safeSetText('temps',`${t.toFixed(2)} s`);
}

function afficherGPS(g){
  safeSetText('gps',`GPS : Lat : ${g.latitude.toFixed(6)} | Lon : ${g.longitude.toFixed(6)} | Alt : ${g.altitude.toFixed(0)} m | Préc. : ${g.accuracy.toFixed(0)} m`);
  safeSetText('compass-display',`Boussole (Cap GPS) : ${g.heading.toFixed(1)}°`);
}

function calculerCible(pos) {
    const bearing = calculerRelèvement(pos, targetCoords);
    const distance_m = calculerDistanceHaversine(pos, targetCoords);
    const distance_km = distance_m / 1000;

    safeSetText('bearing-display', `Relèvement vers la cible : ${bearing.toFixed(1)}° | Distance : ${distance_km.toFixed(3)} km`);
}

function setTargetCoords() {
    const input = document.getElementById('target-coord').value;
    const parts = input.split(',').map(p => parseFloat(p.trim()));
    if (parts.length === 2 && !isNaN(parts[0]) && !isNaN(parts[1])) {
        targetCoords = { latitude: parts[0], longitude: parts[1] };
        alert(`Cible définie : ${targetCoords.latitude}, ${targetCoords.longitude}`);
    } else {
        alert("Format de coordonnées invalide. Utilisez 'lat, lon'.");
    }
}


// ========================
// Contrôles cockpit
// ========================
function demarrerCockpit(){
  if(watchId!==null) return;
  if(!tempsDebut) tempsDebut=Date.now();
  const options={enableHighAccuracy:true,timeout:5000,maximumAge:0};
  
  // L'appel au GPS qui peut déclencher l'invite de permission
  watchId=navigator.geolocation.watchPosition(
    miseAJourVitesse,
    (e)=>{console.error(e);safeSetText('gps',`GPS : Erreur (${e.code})`)},
    options
  );

  document.getElementById('marche').textContent = '⏸️ Pause';
}

function arreterCockpit(){
  if(watchId!==null){navigator.geolocation.clearWatch(watchId); watchId=null;}
  safeSetText('vitesse','PAUSE');
  document.getElementById('marche').textContent = '▶️ Marche';
}

function resetDisplayToZero(){
  safeSetText('vitesse','0.00 km/h'); 
  safeSetText('vitesse-moy','0.00 km/h'); 
  safeSetText('vitesse-max','0.00 km/h');
  safeSetText('vitesse-ms','0.00 m/s | 0 mm/s'); 
  safeSetText('vitesse-verticale-baro','0.0 m/min (0.00 m/s) ↔️ Stable'); 
  safeSetText('vitesse-verticale-gps', '0.0 m/min (0.00 m/s)');
  safeSetText('pression-air', '1013.25 hPa | Alt Baro: 0 m');
  safeSetText('pourcentage','0.00% | 0.00%');
  safeSetText('distance','0.000 km | 0 m | 0 mm'); 
  safeSetText('distance-cosmique','0.000 s lumière | 0.000e+00 al');
  safeSetText('temps','0.00 s'); 
}

function resetCockpit(){
  arreterCockpit(); positionPrecedente=null; vitesseMax=0; vitesses=[]; distanceTotale=0; tempsDebut=null;
  pressionAtmospherique = 1013.25; vitesseVerticaleBaro = 0; 

  resetDisplayToZero();
  safeSetText('gps','GPS : --');
  safeSetText('compass-display','Boussole (Cap GPS) : --°');
  safeSetText('bearing-display', 'Relèvement vers la cible : --° | Distance : -- km');

  if(marker && map) map.removeLayer(marker);
  map = null; 
}

// Fonction de la carte (Laisser vide pour éviter les erreurs)
function initialiserCarte(pos) {}
function mettreAJourCarte(pos) {}
function drawMinecraftClock() {} // Laisser vide

// ========================
// Écouteurs d'événements
// ========================
document.addEventListener('DOMContentLoaded', () => {
    // Écouteur pour le bouton MARCHE / PAUSE
    const marcheButton = document.getElementById('marche');
    if (marcheButton) {
        marcheButton.addEventListener('click', () => {
            if (watchId === null) {
                demarrerCockpit();
                marcheButton.textContent = '⏸️ Pause';
            } else {
                arreterCockpit();
                marcheButton.textContent = '▶️ Marche';
            }
        });
    }

    // Écouteur pour le bouton RÉINITIALISER
    const resetButton = document.getElementById('reset');
    if (resetButton) {
        resetButton.addEventListener('click', resetCockpit);
    }

    // Écouteur pour le bouton MODE SOUTERRAIN
    const toggleSouterrainButton = document.getElementById('toggle-souterrain');
    if (toggleSouterrainButton) {
        toggleSouterrainButton.addEventListener('click', () => {
            modeSouterrainActif = !modeSouterrainActif;
            toggleSouterrainButton.textContent = `🚫 Mode Souterrain : ${modeSouterrainActif ? 'ON' : 'OFF'}`;
            if (modeSouterrainActif) {
                arreterCockpit();
            } else {
                marcheButton.textContent = '▶️ Marche';
            }
        });
    }

    resetCockpit();
});
