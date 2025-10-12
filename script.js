// =====================================================================================
// COCKPIT COSMIQUE - SCRIPT.JS COMPLET POUR APP MOBILE VERTICAL
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
let targetCoords = { latitude: 48.8584, longitude: 2.2945 };

// Constantes physiques
const VITESSE_LUMIERE = 299792458; // m/s
const VITESSE_SON = 343; // m/s
const R_TERRE = 6371e3; // m
const ANNEE_LUMIERE_SECONDES = 3600*24*365.25;
const MASSE_EXPLORATEUR = 70; // kg

// ========================
// UTILITAIRES
// ========================
function safeSetText(id, text){const e=document.getElementById(id);if(e)e.textContent=text;}
function toRadians(deg){return deg*Math.PI/180;}
function toDegrees(rad){return rad*180/Math.PI;}

function calculerDistanceHaversine(p1,p2){
    const φ1=toRadians(p1.latitude);
    const φ2=toRadians(p2.latitude);
    const Δφ=toRadians(p2.latitude-p1.latitude);
    const Δλ=toRadians(p2.longitude-p1.longitude);
    const a=Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
    const c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
    return R_TERRE*c;
}

// ========================
// VITESSE 3D + DISTANCE
// ========================
function miseAJourVitesse3D(pos){
    const gps={
        latitude: pos.coords.latitude,
        longitude: pos.coords.longitude,
        altitude: pos.coords.altitude||0,
        accuracy: pos.coords.accuracy,
        timestamp: pos.timestamp
    };
    let vitesse_ms=0;
    if(positionPrecedente){
        const dt=(gps.timestamp-positionPrecedente.timestamp)/1000;
        if(dt>0){
            const dh=calculerDistanceHaversine(gps,positionPrecedente);
            const dz=gps.altitude-positionPrecedente.altitude;
            const d3d=Math.sqrt(dh*dh+dz*dz);
            vitesse_ms=d3d/dt;
            distanceTotale+=d3d;
            vitesses.push(vitesse_ms*3.6);
            if(vitesses.length>30) vitesses.shift();
            vitesseMax=Math.max(vitesseMax,Math.max(...vitesses));
        }
    }
    positionPrecedente=gps;

    const moyenne_kmh=vitesses.length?vitesses.reduce((a,b)=>a+b,0)/vitesses.length:0;
    safeSetText('vitesse',`Vitesse instantanée : ${(vitesse_ms*3.6).toFixed(2)} km/h`);
    safeSetText('vitesse-moy',`Vitesse moyenne : ${moyenne_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
    safeSetText('vitesse-ms',`Vitesse : ${vitesse_ms.toFixed(2)} m/s | ${(vitesse_ms*1000).toFixed(0)} mm/s`);
    safeSetText('pourcentage',`% Lumière : ${(vitesse_ms/VITESSE_LUMIERE*100).toExponential(2)}% | % Son : ${(vitesse_ms/VITESSE_SON*100).toFixed(2)}%`);

    const km=distanceTotale/1000; const m=distanceTotale; const mm=m*1000;
    const secLumiere=m/VITESSE_LUMIERE; const al=secLumiere/ANNEE_LUMIERE_SECONDES;
    safeSetText('distance',`Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm`);
    safeSetText('distance-cosmique',`Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
    safeSetText('gps',`GPS : Lat : ${gps.latitude.toFixed(6)} | Lon : ${gps.longitude.toFixed(6)} | Alt : ${gps.altitude.toFixed(1)} m | Préc. : ${gps.accuracy?.toFixed(1)||'--'} m`);
}

// ========================
// GESTION COCKPIT
// ========================
function demarrerCockpit(){
    if(watchId!==null) return;
    if(tempsDebut===null){tempsDebut=Date.now();distanceTotale=0;vitesseMax=0;vitesses=[];}
    watchId=navigator.geolocation.watchPosition(miseAJourVitesse3D,error=>{console.error(error);safeSetText('gps','GPS : Erreur');},{enableHighAccuracy:true,timeout:5000,maximumAge:0});
}
function arreterCockpit(){if(watchId!==null){navigator.geolocation.clearWatch(watchId);watchId=null;}safeSetText('vitesse','Vitesse instantanée : **ARRÊT**');}
function resetCockpit(){arreterCockpit();positionPrecedente=null;vitesseMax=0;vitesses=[];distanceTotale=0;tempsDebut=null;
safeSetText('temps','Temps : 0.00 s');
safeSetText('vitesse','Vitesse instantanée : -- km/h');
safeSetText('vitesse-moy','Vitesse moyenne : -- km/h');
safeSetText('vitesse-max','Vitesse max : -- km/h');
safeSetText('vitesse-ms','Vitesse : -- m/s | -- mm/s');
safeSetText('pourcentage','% Lumière : --% | % Son : --%');
safeSetText('distance','Distance : -- km | -- m | -- mm');
safeSetText('distance-cosmique','Distance cosmique : -- s lumière | -- al');
safeSetText('gps','GPS : --');}
        
