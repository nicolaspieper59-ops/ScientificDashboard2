let watchId = null, positionPrecedente = null, vitesseMax = 0, vitesses = [], distanceTotale = 0;
// Objet pour stocker l'état des capteurs physiques (pour affichage unifié)
let capteursEtat = {
  niveauBulle: '--',
  lumiere: '--',
  son: '--',
  magnetisme: '--'
};

// ========================
// UTILS BASE TEXT & DOM
// ========================
const _baseTexts = {};
function safeSetText(id, text) { const e = document.getElementById(id); if (e) e.textContent = text; }
function setElementTextBase(id, base) { _baseTexts[id] = base; const e = document.getElementById(id); if (e) e.textContent = base; }
function getElementText(id) { const e = document.getElementById(id); return e ? e.textContent : _baseTexts[id] || ''; }

// Note: appendText n'est plus utilisé pour les capteurs car on utilise l'objet capteursEtat pour un affichage propre.

// ========================
// GPS / VITESSE
// ========================
function demarrerCockpit(){
  if(!("geolocation" in navigator)){safeSetText('gps','GPS non disponible');return;}
  if(watchId!==null)return;
  // Utilise la vitesse native (pos.coords.speed) si disponible, sinon la calcule.
  watchId=navigator.geolocation.watchPosition(pos=>{
    const gps={latitude:pos.coords.latitude,longitude:pos.coords.longitude,altitude:pos.coords.altitude,accuracy:pos.coords.accuracy,timestamp:pos.timestamp,speed:pos.coords.speed};
    const vitesse=(typeof gps.speed==='number' && gps.speed!==null && gps.speed>=0)?gps.speed*3.6:calculerVitesse(gps);
    
    if(vitesse>=0 && vitesse<300){
      vitesseMax=Math.max(vitesseMax,vitesse);vitesses.push(vitesse);
      afficherVitesse(vitesse);afficherDistance();afficherPourcentage(vitesse);afficherGPS(gps);
    }
  },err=>{console.warn('GPS refusé ou erreur',err);safeSetText('gps','Erreur GPS: '+err.code);},{enableHighAccuracy:true,maximumAge:0,timeout:10000});
}
function arreterCockpit(){
  if(watchId!==null)navigator.geolocation.clearWatch(watchId);
  watchId=null;
  positionPrecedente=null;
  vitesses=[];
  distanceTotale=0;
  vitesseMax=0;
  // Réinitialisation des capteurs virtuels/physiques
  capteursEtat = {niveauBulle: '--', lumiere: '--', son: '--', magnetisme: '--'};
  afficherCapteurs();
}
function resetVitesseMax(){vitesseMax=0;}

// ========================
// CALCULS
// ========================
function calculerVitesse(gps){
  if(!positionPrecedente){positionPrecedente=gps;return 0;}
  const dt=(gps.timestamp-positionPrecedente.timestamp)/1000;
  const d=calculerDistance(gps,positionPrecedente);
  if(!Number.isFinite(d)||d<0){positionPrecedente=gps;return 0;}
  distanceTotale+=d;
  positionPrecedente=gps;
  return dt>0?(d/dt)*3.6:0; // Vitesse en km/h
}
function calculerDistance(p1,p2){
  const R=6371e3,
  φ1=p1.latitude*Math.PI/180,φ2=p2.latitude*Math.PI/180,
  Δφ=(p2.latitude-p1.latitude)*Math.PI/180,
  Δλ=(p2.longitude-p1.longitude)*Math.PI/180,
  a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2,
  c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  return R*c; // Distance en mètres
}

// ========================
// AFFICHAGE GPS / VITESSE
// ========================
function afficherVitesse(v){
  const m=vitesses.length?vitesses.reduce((a,b)=>a+b,0)/vitesses.length:0;
  const mps=v/3.6,mmps=mps*1000;
  safeSetText('vitesse',`Temps : ${new Date().toLocaleTimeString()} | Vitesse : ${v.toFixed(2)} km/h | Moyenne : ${m.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
}
function afficherDistance(){
  const km=distanceTotale/1000,m=distanceTotale,
  secLumiere=m/299792458,
  al=secLumiere/(3600*24*365.25);
  safeSetText('distance',`Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | Cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}
function afficherPourcentage(v){
  const mps=v/3.6;
  safeSetText('pourcentage',`% Lumière : ${(mps/299792458*100).toExponential(2)}% | % Son : ${(mps/343*100).toFixed(2)}%`);
}
function afficherGPS(g){
  safeSetText('gps',`Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${g.altitude?.toFixed(0)??'--'} m | Précision GPS : ${g.accuracy?.toFixed(0)??'--'} m`);
}

// ========================
// AFFICHAGE CAPTEURS PHYSIQUES
// ========================
function afficherCapteurs() {
  safeSetText('capteurs', 
    `Niveau à bulle : ${capteursEtat.niveauBulle}° | Lumière : ${capteursEtat.lumiere} lux | Son : ${capteursEtat.son} dB | Magnétisme : ${capteursEtat.magnetisme} µT`
  );
}

async function activerCapteurs(){
  afficherCapteurs(); // Initialisation

  // Lumière (AmbientLightSensor)
  if('AmbientLightSensor' in window){try{const s=new AmbientLightSensor();s.addEventListener('reading',()=>{
    capteursEtat.lumiere = Math.round(s.illuminance);
    afficherCapteurs();
  });s.start();}catch(e){console.warn('AmbientLightSensor refusé ou erreur', e); capteursEtat.lumiere = 'Err';}}
  
  // Micro (Niveau Sonore - nécessite HTTPS pour le micro)
  if(navigator.mediaDevices?.getUserMedia){
    try{
      const stream=await navigator.mediaDevices.getUserMedia({audio:true});
      const ctx=new (window.AudioContext||window.webkitAudioContext)();
      const analyser=ctx.createAnalyser();
      const source=ctx.createMediaStreamSource(stream);
      source.connect(analyser);
      analyser.fftSize=2048;
      const data=new Uint8Array(analyser.frequencyBinCount);
      function loop(){
        analyser.getByteFrequencyData(data);
        const rms = Math.sqrt(data.reduce((a, b) => a + b * b, 0) / data.length);
        const dB = 20 * Math.log10(rms > 0 ? rms : 1);
        capteursEtat.son = Math.round(dB);
        afficherCapteurs();
        requestAnimationFrame(loop);
      }
      loop();
    }catch(e){console.warn('Micro refusé ou erreur', e); capteursEtat.son = 'Err';}
  }
  
  // Orientation (Niveau à bulle - nécessite HTTPS sur certains navigateurs)
  if(typeof DeviceOrientationEvent!=='undefined'){
    const updateOrientation = e => {
      if(e.beta != null) {
        capteursEtat.niveauBulle = e.beta.toFixed(1);
        afficherCapteurs();
      }
    };
    if(typeof DeviceOrientationEvent.requestPermission==='function'){
      try{
        const p=await DeviceOrientationEvent.requestPermission();
        if(p==='granted')window.addEventListener('deviceorientation', updateOrientation);
      }catch(e){console.warn('DeviceOrientation refusé', e);}
    }else{
      window.addEventListener('deviceorientation', updateOrientation);
    }
  }
  
  // Magnétomètre (Nécessite HTTPS)
  if('Magnetometer' in window){
    try{
      const m=new Magnetometer();
      m.addEventListener('reading',()=>{
        capteursEtat.magnetisme = Math.round(Math.sqrt(m.x**2+m.y**2+m.z**2));
        afficherCapteurs();
      });
      m.start();
    }catch(e){console.warn('Magnetometer refusé ou erreur', e); capteursEtat.magnetisme = 'Err';}
  }
}

// ========================
// HORLOGE & MEDAILLON
// ========================
function activerHorloge(){
  const h=document.getElementById('horloge');
  function loop(){
    const t=new Date();
    h.textContent=`⏰ ${t.getHours().toString().padStart(2,'0')}:${t.getMinutes().toString().padStart(2,'0')}:${t.getSeconds().toString().padStart(2,'0')}`;
    requestAnimationFrame(loop);
  }
  requestAnimationFrame(loop);
}
function afficherMedaillon(){
  const m=document.getElementById('medaillon');if(!m)return;
  m.innerHTML='';
  const c=document.createElement('canvas');c.width=300;c.height=300;m.appendChild(c);
  const ctx=c.getContext('2d');
  
  ctx.fillStyle='#000'; 
  ctx.fillRect(0,0,c.width,c.height);
  
  // Design simpliste pour la démonstration
  ctx.strokeStyle = '#0ff';
  ctx.lineWidth = 5;
  ctx.beginPath();
  ctx.arc(150, 150, 120, 0, 2 * Math.PI);
  ctx.stroke();

  ctx.fillStyle='#ffd700'; // Soleil
  ctx.beginPath();
  ctx.arc(150, 110, 15, 0, 2*Math.PI);
  ctx.fill();
  
  ctx.fillStyle='#ccc'; // Lune/Terre
  ctx.beginPath();
  ctx.arc(150, 200, 12, 0, 2*Math.PI);
  ctx.fill();
}

// ========================
// INIT CLICK
// ========================
document.addEventListener('DOMContentLoaded', () => {
  // Récupère le texte initial du HTML et le stocke comme base
  const defaultGPS = document.getElementById('gps').textContent;
  const defaultCapteurs = document.getElementById('capteurs').textContent;
  setElementTextBase('gps', defaultGPS);
  setElementTextBase('capteurs', defaultCapteurs);
  
  // Initialisation des éléments statiques
  afficherMedaillon();
  activerHorloge();

  const btnMarche = document.getElementById('marche');
  const btnReset = document.getElementById('reset');

  // Événement Démarrer/Arrêter
  btnMarche.addEventListener('click', async () => {
    if (watchId === null) {
      demarrerCockpit();
      await activerCapteurs();
      btnMarche.textContent = '⏹️ Arrêter';
    } else {
      arreterCockpit();
      btnMarche.textContent = '▶️ Marche';
      // Réinitialiser les affichages
      afficherVitesse(0);
      afficherDistance();
      afficherPourcentage(0);
      safeSetText('gps', defaultGPS);
    }
  });

  // Événement Réinitialiser vitesse max
  btnReset.addEventListener('click', () => { 
    resetVitesseMax(); 
    const vitesseElement = document.getElementById('vitesse');
    if (vitesseElement) {
      // Met à jour uniquement la valeur "Max" dans la chaîne de texte existante
      vitesseElement.textContent = vitesseElement.textContent.replace(/Max : [\d\.]+ km\/h/, 'Max : 0.00 km/h');
    }
  });
});
