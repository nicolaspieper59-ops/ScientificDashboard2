let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let distanceTotale = 0;
let vitesseMax = 0;
let destination = { latitude: null, longitude: null };
let simulationActive = false;

// Simulation physique
let masseSimulee = 70; // kg
let vitesseSimulee = 0; // m/s

/* ========================
   UTILITAIRES DOM
======================== */
function safeSetText(id,text){
    const el = document.getElementById(id);
    if(el) el.textContent = text;
}

/* ========================
   GPS
======================== */
export function demarrerCockpit(){

    // Position instantanée
    navigator.geolocation.getCurrentPosition(
        pos => traiterPosition(pos.coords,pos.timestamp),
        err => console.error(err)
    );

    // Position continue
    if(watchId===null){
        watchId = navigator.geolocation.watchPosition(
            pos => traiterPosition(pos.coords,pos.timestamp),
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
    if(watchId!==null){
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

export function definirDestination(lat,lon){
    destination.latitude = lat;
    destination.longitude = lon;
}

/* ========================
   TRAITEMENT POSITION
======================== */
function traiterPosition(coords,timestamp){
    const lat = coords.latitude;
    const lon = coords.longitude;
    const alt = coords.altitude ?? 0;

    let vitesse = coords.speed!==null ? coords.speed*3.6 : calculerVitesse(coords,timestamp);
    if(vitesse>=0 && vitesse<300){
        vitesseMax = Math.max(vitesseMax,vitesse);
        vitesses.push(vitesse);
        afficherVitesse(vitesse);
        afficherDistance();
        afficherPourcentage(vitesse);
    }

    safeSetText('gps',`Latitude : ${lat.toFixed(6)} | Longitude : ${lon.toFixed(6)} | Altitude : ${alt.toFixed(1)} m`);
}

function calculerVitesse(coords,timestamp){
    if(!positionPrecedente){
        positionPrecedente = { coords,timestamp };
        return 0;
    }
    const dt = (timestamp - positionPrecedente.timestamp)/1000;
    const d = calculerDistance(coords,positionPrecedente.coords);
    distanceTotale += d;
    positionPrecedente = { coords,timestamp };
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
   AFFICHAGE
======================== */
function afficherVitesse(v){
    const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
    safeSetText('vitesse',`Vitesse : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} | Max : ${vitesseMax.toFixed(2)}`);
}

function afficherDistance(){
    safeSetText('distance',`Distance : ${(distanceTotale/1000).toFixed(3)} km`);
}

function afficherPourcentage(v){
    const mps = v/3.6;
    const pctL = (mps/299792458)*100;
    const pctS = (mps/343)*100;
    safeSetText('pourcentage',`% Lumière : ${pctL.toExponential(2)}% | % Son : ${pctS.toFixed(2)}%`);
}

/* ========================
   CAPTEURS
======================== */
function activerCapteurs(){
    const cap = document.getElementById('capteurs');
    cap.textContent = '';

    if('AmbientLightSensor' in window){
        try{
            const light = new AmbientLightSensor();
            light.addEventListener('reading',()=>cap.textContent=`Lux : ${Math.round(light.illuminance)}`);
            light.start();
        }catch{}
    }

    if(navigator.mediaDevices && navigator.mediaDevices.getUserMedia){
        navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
            const audioCtx = new AudioContext();
            const analyser = audioCtx.createAnalyser();
            const source = audioCtx.createMediaStreamSource(stream);
            source.connect(analyser);
            const data = new Uint8Array(analyser.frequencyBinCount);
            (function loop(){
                analyser.getByteFrequencyData(data);
                const dB = 20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);
                cap.textContent+=` | dB : ${dB.toFixed(0)} | Hz : ${analyser.frequencyBinCount}`;
                requestAnimationFrame(loop);
            })();
        }).catch(()=>{});
    }

    if('DeviceOrientationEvent' in window){
        window.addEventListener('deviceorientation', e=>{
            if(e.beta!=null) cap.textContent+=` | Niveau : ${e.beta.toFixed(1)}°`;
        });
    }

    if('Magnetometer' in window){
        try{
            const mag = new Magnetometer();
            mag.addEventListener('reading',()=>cap.textContent+=` | Magnétisme : ${Math.round(Math.sqrt(mag.x**2+mag.y**2+mag.z**2))} µT`);
            mag.start();
        }catch{}
    }
}

/* ========================
   Boussole, Horloge, Médaillon, Météo, Grandeurs
======================== */
function activerB
           
