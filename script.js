// =====================================================================================
// Cockpit Cosmique - script.js
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
let modeSouterrainActif = false;
let deviceOrientationListener = null;
let bleDevice = null;
let bleValue = '--';
let targetCoords = { latitude: 48.8584, longitude: 2.2945 };

// Paramètres scientifiques
const VITESSE_LUMIERE = 299792458; // m/s
const VITESSE_SON = 343;
const R_TERRE = 6371e3;
const ANNEE_LUMIERE_SECONDES = 3600*24*365.25;
const MASSE_EXPLORATEUR = 70; // kg

window.settings = {
    showPhysique: true,
    showChimie: true,
    showSVT: true,
    showRituel: true,
    showVitesse: true,
    showAstronomie: true,
    showCapteurs: true,
    showCarte: true
};

// ========================
// UTILS
// ========================
function safeSetText(id,text){const e=document.getElementById(id);if(e)e.textContent=text;}
function toRadians(d){return d*Math.PI/180;}
function toDegrees(r){return r*180/Math.PI;}

// ========================
// GPS & VITESSE 3D + Dead Reckoning
// ========================
function calculerDistanceHaversine(p1,p2){
  const φ1=toRadians(p1.latitude),φ2=toRadians(p2.latitude);
  const Δφ=toRadians(p2.latitude-p1.latitude),Δλ=toRadians(p2.longitude-p1.longitude);
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  return R_TERRE*c;
}

function miseAJourVitesse(pos){
  if(modeSouterrainActif) return;
  const gps={ latitude:pos.coords.latitude, longitude:pos.coords.longitude, altitude:pos.coords.altitude, accuracy:pos.coords.accuracy, timestamp:pos.timestamp, speed:pos.coords.speed, heading:pos.coords.heading };
  let vitesse_ms=0;
  if(positionPrecedente){
    const dt=(gps.timestamp-positionPrecedente.timestamp)/1000;
    const dist=calculerDistanceHaversine(gps,positionPrecedente);
    distanceTotale+=dist;
    vitesse_ms=(gps.speed>=0)?gps.speed:dist/dt;
  }
  positionPrecedente=gps;
  const vitesse_kmh=vitesse_ms*3.6;
  if(vitesse_kmh>=0){
    vitesseMax=Math.max(vitesseMax,vitesse_kmh);
    vitesses.push(vitesse_kmh);
    if(vitesses.length>30) vitesses.shift();
    afficherVitesse(vitesse_ms,vitesse_kmh);
    afficherDistance();
    afficherTemps();
    updateNavigation(gps,targetCoords);
    updateCompass(gps.heading);
    afficherGPS(gps);
  }
}

function afficherVitesse(v_ms,v_kmh){
  if(!window.settings.showVitesse) return;
  const mmps=v_ms*1000;
  const moyenne_kmh=vitesses.length?vitesses.reduce((a,b)=>a+b,0)/vitesses.length:0;
  safeSetText('vitesse',`Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy',`Vitesse moyenne : ${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms',`Vitesse : ${v_ms.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
  safeSetText('pourcentage',`% Lumière : ${(v_ms/VITESSE_LUMIERE*100).toExponential(2)}% | % Son : ${(v_ms/VITESSE_SON*100).toFixed(2)}%`);
}

function afficherDistance(){
  const km=distanceTotale/1000,m=distanceTotale,mm=m*1000;
  const secLumiere=m/VITESSE_LUMIERE;
  const al=secLumiere/ANNEE_LUMIERE_SECONDES;
  safeSetText('distance',`Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm`);
  safeSetText('distance-cosmique',`Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps(){
  if(!tempsDebut) return;
  const t=(Date.now()-tempsDebut)/1000;
  safeSetText('temps',`Temps : ${t.toFixed(2)} s`);
}

function afficherGPS(g){
  safeSetText('gps',`GPS : Lat ${g.latitude.toFixed(6)} | Lon ${g.longitude.toFixed(6)} | Alt ${g.altitude?.toFixed(0)||'--'} m | Préc. ${g.accuracy?.toFixed(0)||'--'} m`);
}

// ========================
// MAP & NAVIGATION
// ========================
let map = L.map('map').setView([48.8584,2.2945],15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);
let marker = L.marker([48.8584,2.2945]).addTo(map);

function getBearing(p1,p2){const φ1=toRadians(p1.latitude),λ1=toRadians(p1.longitude);const φ2=toRadians(p2.latitude),λ2=toRadians(p2.longitude);const y=Math.sin(λ2-λ1)*Math.cos(φ2);const x=Math.cos(φ1)*Math.sin(φ2)-Math.sin(φ1)*Math.cos(φ2)*Math.cos(λ2-λ1);return (toDegrees(Math.atan2(y,x))+360)%360;}

function updateNavigation(current,target){
  const bearing=getBearing(current,target);
  const distance_km=calculerDistanceHaversine(current,target)/1000;
  safeSetText('bearing-display',`Relèvement vers la cible : ${bearing.toFixed(1)}° | Distance : ${distance_km.toFixed(2)} km`);
  marker.setLatLng([current.latitude,current.longitude]);
  map.setView([current.latitude,current.longitude]);
}

function updateCompass(heading){
  let deg='--';
  if(typeof heading==='number'&&heading!==null) deg=heading.toFixed(1);
  safeSetText('compass-display',`Boussole (Nord Vrai) : ${deg}°`);
}

// ========================
// BLE/UWB
// ========================
const BLE_SERVICE_UUID='0000xxxx-0000-1000-8000-00805f9b34fb';
const BLE_CHAR_UUID='0000yyyy-0000-1000-8000-00805f9b34fb';
async function connectBLE(){
  try{
    bleDevice=await navigator.bluetooth.requestDevice({filters:[{services:[BLE_SERVICE_UUID]}]});
    const server=await bleDevice.gatt.connect();
    const service=await server.getPrimaryService(BLE_SERVICE_UUID);
    const characteristic=await service.getCharacteristic(BLE_CHAR_UUID);
    const value=await characteristic.readValue();
    parseBLEValue(value);
    characteristic.startNotifications();
    characteristic.addEventListener('characteristicvaluechanged',e=>parseBLEValue(e.target.value));
    alert('BLE/UWB connecté');
  }catch(err){console.error(err);alert('BLE/UWB impossible : '+err);}
}
function parseBLEValue(dataView){
  if(dataView.byteLength>=4){
    const val=dataView.getFloat32(0,true);
    bleValue=val.toFixed(2);
    safeSetText('reseau',`BLE/UWB : ${bleValue}`);
  }
}

// ========================
// Cockpit Contrôle
// ========================
function demarrerCockpit(){
  if(watchId!==null) return;
  if(!tempsDebut) tempsDebut=Date.now();
  const options={enableHighAccuracy:true,timeout:5000,maximumAge:0};
  watchId=navigator.geolocation.watchPosition(miseAJourVitesse,err=>{console.error(err);safeSetText('gps',`GPS : Erreur (${err.code})`);},options);
}
function arreterCockpit(){if(watchId!==null){navigator.geolocation.clearWatch(watchId);watchId=null;} safeSetText('vitesse','Vitesse instantanée : **ARRÊT**');}
function resetCockpit(){arreterCockpit();positionPrecedente=null;vitesseMax=0;vitesses=[];distanceTotale=0;tempsDebut=null; safeSetText('temps','Temps : 0.00 s'); safeSetText('vitesse','Vitesse instantanée : -- km/h'); safeSetText('vitesse-moy','Vitesse moyenne : -- km/h'); safeSetText('vitesse-max','Vitesse max : -- km/h'); safeSetText('vitesse-ms','Vitesse : -- m/s | -- mm/s'); safeSetText('pourcentage','% Lumière : --% | % Son : --%'); safeSetText('distance','Distance : -- km
