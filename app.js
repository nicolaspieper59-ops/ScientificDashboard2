// =====================================================================================
// Full Cockpit Scientifique - GPS + IMU + Astronomie + BLE/UWB
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
let historiqueVitesse = [];
let intervalleSysteme = null;
let modeSouterrainActif = false;

// BLE/UWB générique
let bleDevice = null;
const BLE_SERVICE_UUID = '00001234-0000-1000-8000-00805f9b34fb';
const BLE_CHAR_UUID = '00005678-0000-1000-8000-00805f9b34fb';
let bleValue = null;

// Fusion GPS + IMU (Kalman simplifié)
let state = { x: 0, y: 0, vx: 0, vy: 0 };
const dt = 0.1; // intervalle en sec pour Dead Reckoning

// Réglages utilisateur pour économie batterie
const settings = {
    style: true,
    vitesseInstant: true,
    vitesseMoy: true,
    vitesseMax: true,
    distance: true,
    temps: true,
    astronomie: true,
    imu: true,
    ble: true
};

// Charger les réglages
if(localStorage.getItem('cockpitSettings')){
    Object.assign(settings, JSON.parse(localStorage.getItem('cockpitSettings')));
}

// ========================
// UTILS
// ========================
function safeSetText(id,text){ const e=document.getElementById(id); if(e) e.textContent=text; }
function toRadians(deg){return deg*Math.PI/180;}
function toDegrees(rad){return rad*180/Math.PI;}
function calculerDistanceHaversine(p1,p2){
    const R=6371e3;
    const φ1=toRadians(p1.latitude),φ2=toRadians(p2.latitude);
    const Δφ=toRadians(p2.latitude-p1.latitude);
    const Δλ=toRadians(p2.longitude-p1.longitude);
    const a=Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
    const c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
    return R*c;
}

// ========================
// FUSION GPS + IMU (Dead Reckoning)
// ========================
function updateFusionGPSIMU(accelX, accelY){
    // Position estimée
    state.vx += accelX*dt;
    state.vy += accelY*dt;
    state.x += state.vx*dt;
    state.y += state.vy*dt;
}

// ========================
// MISE A JOUR VITESSE
// ========================
function miseAJourVitesse(pos){
    const gps=pos.coords;
    let vitesse_ms = gps.speed || 0;

    if(positionPrecedente){
        const dtGPS=(pos.timestamp-positionPrecedente.timestamp)/1000;
        const dist = calculerDistanceHaversine(gps,positionPrecedente);
        distanceTotale+=dist;
        if(!vitesse_ms && dtGPS>0) vitesse_ms = dist/dtGPS;
    }

    positionPrecedente=gps;
    const vitesse_kmh=vitesse_ms*3.6;
    vitesseMax=Math.max(vitesseMax,vitesse_kmh);
    vitesses.push(vitesse_kmh);
    historiqueVitesse.push(vitesse_kmh);
    if(vitesses.length>30) vitesses.shift();
    if(historiqueVitesse.length>100) historiqueVitesse.shift();

    // Affichage selon réglages
    if(settings.vitesseInstant) safeSetText('vitesse',`Vitesse instantanée : ${vitesse_kmh.toFixed(2)} km/h`);
    if(settings.vitesseMoy){ const moy=vitesses.reduce((a,b)=>a+b,0)/vitesses.length; safeSetText('vitesse-moy',`Vitesse moyenne : ${moy.toFixed(2)} km/h`);}
    if(settings.vitesseMax) safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
    if(settings.distance){ const km=distanceTotale/1000; safeSetText('distance',`Distance : ${km.toFixed(3)} km`);}
    if(settings.temps && tempsDebut){ const t=(Date.now()-tempsDebut)/1000; safeSetText('temps',`Temps : ${t.toFixed(2)} s`);}

    updateGraphique();
    updateIndicator(vitesse_kmh);
    if(settings.astronomie) activerAstronomie(gps.latitude,gps.longitude);
}

// ========================
// INDICATEUR MARCHE/ARRET
// ========================
function updateIndicator(v){
    let ind=document.getElementById('indicator');
    if(!ind){ ind=document.createElement('div'); ind.id='indicator'; ind.style.width='20px'; ind.style.height='20px'; ind.style.borderRadius='50%'; ind.style.display='inline-block'; ind.style.marginLeft='10px'; document.querySelector('h2').appendChild(ind); }
    ind.style.backgroundColor = watchId?'#0f0':'#f00';
}

// ========================
// GRAPHIQUE HISTORIQUE VITESSE
// ========================
function updateGraphique(){
    let canvas=document.getElementById('graphVitesse');
    if(!canvas){ canvas=document.createElement('canvas'); canvas.id='graphVitesse'; canvas.style.width='100%'; canvas.style.height='150px'; document.querySelector('.card').appendChild(canvas);}
    const ctx=canvas.getContext('2d');
    ctx.clearRect(0,0,canvas.width,canvas.height);
    ctx.beginPath(); ctx.strokeStyle='#007bff';
    const stepX=canvas.width/100;
    const maxV=Math.max(...historiqueVitesse,50);
    historiqueVitesse.forEach((v,i)=>{const x=i*stepX; const y=canvas.height-(v/maxV)*canvas.height; if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);});
    ctx.stroke();
}

// ========================
// DÉMARRER/ARRÊTER/RÉINITIALISER
// ========================
function demarrerCockpit(){
    if(watchId) return;
    if(!tempsDebut) tempsDebut=Date.now();
    watchId = navigator.geolocation.watchPosition(miseAJourVitesse,(err)=>{console.error(err); safeSetText('vitesse','GPS indisponible');},{enableHighAccuracy:true,maximumAge:0,timeout:5000});
    document.getElementById('toggleBtn').textContent='Arrêt';
}
function arreterCockpit(){
    if(watchId) navigator.geolocation.clearWatch(watchId);
    watchId=null; document.getElementById('toggleBtn').textContent='Démarrer';
}
function resetCockpit(){
    arreterCockpit(); positionPrecedente=null; vitesseMax=0; vitesses=[]; distanceTotale=0; tempsDebut=null; historiqueVitesse=[];
    safeSetText('vitesse','Vitesse instantanée : -- km/h');
    safeSetText('vitesse-moy','Vitesse moyenne : -- km/h');
    safeSetText('vitesse-max','Vitesse max : -- km/h');
    safeSetText('distance','Distance : -- km'); safeSetText('temps','Temps : 0.00 s');
    const canvas=document.getElementById('graphVitesse'); if(canvas) canvas.getContext('2d').clearRect(0,0,canvas.width,canvas.height);
    updateIndicator(0);
}

// ========================
// ASTRONOMIE
// ========================
function activerAstronomie(lat,lon){
    const now=new Date(); calculerHeuresSolaires(now,lat,lon); const lune=calculerLune(now); safeSetText('lune-phase',`Phase Lune : ${lune.phase}`); }
function calculerHeuresSolaires(now,lat,lon){
    const Jd=(now.getTime()/(24*60*60*1000))+2440587.5;
    const T=(Jd-2451545.0)/36525;
    const M=357.52911 + 35999.05029*T - 0.0001537*T*T;
    const M_rad=toRadians(M%360);
    const L0=280.46646 + 36000.76983*T + 0.0003032*T*T;
    const L0_mod=L0%360;
    const C=(1.9146-0.004817*T)*Math.sin(M_rad)+(0.01994-0.000101*T)*Math.sin(2*M_rad);
    const lambda=L0_mod+C; const lambda_rad=toRadians(lambda);
    const epsilon=23.439291-0.013004*T; const epsilon_rad=toRadians(epsilon);
    const alpha_rad=Math.atan2(Math.cos(epsilon_rad)*Math.sin(lambda_rad),Math.cos(lambda_rad));
    const alpha_deg=toDegrees(alpha_rad); const alpha_norm=alpha_deg<0?alpha_deg+360:alpha_deg;
    let EqT_deg=L0_mod-alpha_norm; if(EqT_deg>180) EqT_deg-=360; if(EqT_deg<-180) EqT_deg+=360;
    const EqT_sec=EqT_deg*240; safeSetText('equation-temps',`Eq Temps
