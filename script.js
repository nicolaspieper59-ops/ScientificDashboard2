// Assurez-vous d'avoir bien initialisé ces variables globales au début de votre fichier
// let positionPrecedente = null; 
// let tempsDebut = null;
// const VITESSE_LUMIERE = 299792458;
// const VITESSE_SON = 343;
// const R_TERRE = 6371e3;
// let vitesses = []; 
// let distanceTotale = 0;

// ========================
// GPS & vitesse (Version sans baromètre)
// ========================
function miseAJourVitesse(pos){
  const now = Date.now();
  if(!tempsDebut) tempsDebut = now;

  // Filtre mode souterrain
  if(modeSouterrainActif) {
    resetDisplayToZero(); // S'assure que tout est à 0.00
    safeSetText('gps',`GPS : Mode Souterrain - Données Figées`);
    return;
  }
  
  // Filtre de précision (Optionnel, mais recommandé)
  if(pos.coords.accuracy > 25) {
    safeSetText('gps', `GPS : Signal faible (${pos.coords.accuracy.toFixed(0)} m). Attente de précision...`);
    return; 
  }

  const gps={
    latitude:pos.coords.latitude,
    longitude:pos.coords.longitude,
    altitude:pos.coords.altitude||0, // Le || 0 est crucial
    accuracy:pos.coords.accuracy,
    speed:pos.coords.speed||0,
    heading:pos.coords.heading
  };

  let vitesse_ms=0;
  let vitesse_verticale_gps = 0; // Vitesse verticale GPS

  if(positionPrecedente){
    const dt=(pos.timestamp-positionPrecedente.timestamp)/1000;
    const dist=calculerDistanceHaversine(gps,positionPrecedente);
    distanceTotale+=dist;
    
    // Calcul de la Vitesse Horizontale
    vitesse_ms=(gps.speed>0 && gps.speed<VITESSE_LUMIERE) ? gps.speed : (dt > 0 ? dist/dt : 0);
    
    // Calcul de la Vitesse Verticale GPS
    const deltaAltitudeGPS = gps.altitude - positionPrecedente.altitude;
    vitesse_verticale_gps = (dt > 0) ? deltaAltitudeGPS / dt : 0;
  } else {
    // Initialisation
    // Assurez-vous que initialiserCarte existe et est appelée ici
    // initialiserCarte(gps); 
  }
  
  positionPrecedente=gps; 

  const vitesse_kmh=vitesse_ms*3.6;
  vitesseMax=Math.max(vitesseMax,vitesse_kmh);
  vitesses.push(vitesse_kmh); if(vitesses.length>30) vitesses.shift();
  
  // Affichage
  afficherVitesse(vitesse_kmh, vitesse_ms, vitesse_verticale_gps); 
  afficherDistance();
  afficherTemps();
  afficherGPS(gps);
  // mettreAJourCarte(gps);
  // calculerCible(gps);
}

function afficherVitesse(v_kmh,v_ms, v_verticale_gps){ // MODIFIÉ (retrait de v_verticale_baro)
  const moyenne_kmh=vitesses.reduce((a,b)=>a+b,0)/vitesses.length||0;
  
  const v_vert_gps_m_min = v_verticale_gps * 60;
  const direction_gps = v_verticale_gps > 0 ? '↗️ Montée' : (v_verticale_gps < 0 ? '↘️ Descente' : '↔️ Stable');

  safeSetText('vitesse',`Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy',`Vitesse moyenne (30s) : ${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms',`Vitesse Horizontale : ${v_ms.toFixed(2)} m/s | ${(v_ms*1000).toFixed(0)} mm/s`);
  
  // Affichage GPS vertical uniquement
  safeSetText('vitesse-verticale',`Vitesse Verticale GPS : ${v_vert_gps_m_min.toFixed(1)} m/min (${v_verticale_gps.toFixed(2)} m/s) ${direction_gps}`);
  
  // Vous pouvez réutiliser cette ligne si vous avez retiré les autres champs barométriques du HTML.
  // Si vous voulez conserver les champs barométriques vides dans le HTML, assurez-vous de les réinitialiser dans resetDisplayToZero().
  // safeSetText('pression-air', 'Pression : -- hPa | Alt Baro: -- m');
  // safeSetText('vitesse-verticale-baro', 'Vitesse Verticale Baro : -- m/min (-- m/s)');

  safeSetText('pourcentage',`% Lumière : ${(v_ms/VITESSE_LUMIERE*100).toExponential(2)}% | % Son : ${(v_ms/VITESSE_SON*100).toFixed(2)}%`);
      }
