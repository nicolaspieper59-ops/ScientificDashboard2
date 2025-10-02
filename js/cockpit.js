let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesses = [];
let vitesseMax = 0;
let destination = { latitude: null, longitude: null };

// Simulation
let masseSimulee = 70; // kg
let vitesseSimulee = 0; // m/s
let simulationActive = false;

/* ========================
   UTILITAIRES DOM
======================== */
function safeSetText(id, text){
    const el = document.getElementById(id);
    if(el) el.textContent = text;
}

/* ========================
   GPS & VITESSE
======================== */
export function demarrerCockpit(){
    if(!("geolocation" in navigator)){
        safeSetText('gps','GPS non disponible');
        return;
    }

    // Position instantanée
    navigator.geolocation.getCurrentPosition(
        pos => traiterPosition(pos.coords, pos.timestamp),
        err => console.error(err)
    );

    // Position continue
    if(watchId === null){
        watchId = navigator.geolocation.watchPosition(
            pos => traiterPosition(pos.coords, pos.timestamp),
            err => console.error(err),
            { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
        );
    }

    activerCapteurs();
    activerBoussole();
    activerHorloge();
    afficherMedaillon();
    chargerMeteo();
    afficherGrandeurs();
}

export function arreterCockpit(){
    if(watchId !== null){
        navigator.geolocation.clearWatch(watchId);
        watchId = null;
    }
    positionPrecedente = null;
    distanceTotale = 0;
    vitesses = [];
    vitesseMax = 0;
}

export function resetVitesseMax(){
    vitesseMax = 0;
}

/* ========================
   TRAITEMENT POSITION
======================== */
function traiterPosition(coords, timestamp){
    const lat = coords.latitude;
    const lon = coords.longitude;
    const alt = coords.altitude ?? 0;

    let vitesse = coords.speed !== null ? coords.speed*3.6 : calculerVitesse(coords, timestamp);
    if(vitesse >=0 && vitesse<300){
        vitesseMax = Math.max(vitesseMax, vitesse);
        vitesses.push(vitesse);
        afficherVitesse(vitesse);
        afficherDistance();
        afficherPourcentage(vitesse);
    }

    safeSetText('gps', `Latitude : ${lat.toFixed(6)} | Longitude : ${lon.toFixed(6)} | Altitude : ${alt.toFixed(1)} m`);
}

function calculerVitesse(coords, timestamp){
    if(!positionPrecedente){
        positionPrecedente = { coords, timestamp };
        return 0;
    }
    const dt = (timestamp - positionPrecedente.timestamp)/1000;
    const d = calculerDistance(coords, positionPrecedente.coords);
    distanceTotale += d;
    positionPrecedente = { coords, timestamp };
    return dt>0 ? (d/dt)*3.6 : 0;
}

function calculerDistance(a,b){
    const R = 6371e3;
    const φ1 = a.latitude*Math.PI/180;
    const φ2 = b.latitude*Math.PI/180;
    const Δφ = (b.latitude-a.latitude)*Math.PI/180;
    const Δλ = (b.longitude-a.longitude)*Math.PI/180;
    const c = 2*Math.atan2(Math.sqrt(Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2), Math.sqrt(1-(Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2)));
    return R*c;
}

/* ========================
   AFFICHAGE VITESSE / DISTANCE
======================== */
function afficherVitesse(v){
    const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0
