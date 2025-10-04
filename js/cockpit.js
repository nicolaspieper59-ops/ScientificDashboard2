let watchId = null;
let prevPos = null;
let distanceTotale = 0;
let vitesses = [];

// Met à jour le DOM
function set(id, txt) {
  const el = document.getElementById(id);
  if(el) el.textContent = txt;
}

// Démarrer la collecte de données
export function demarrer() {
  if(!navigator.geolocation) return set("gps","GPS non disponible");

  distanceTotale = 0; vitesses = []; prevPos = null;

  // GPS / Vitesse
  watchId = navigator.geolocation.watchPosition(onPosition, onError, {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 20000
  });

  // Orientation et mouvement
  if(window.DeviceOrientationEvent) {
    window.addEventListener('deviceorientation', e=>{
      const alpha = e.alpha?.toFixed(1) ?? "--";
      const beta = e.beta?.toFixed(1) ?? "--";
      const gamma = e.gamma?.toFixed(1) ?? "--";
      set("orientation", `Orientation : α=${alpha} β=${beta} γ=${gamma} (Magnétomètre + Gyroscope)`);
    });
  }

  if(window.DeviceMotionEvent) {
    window.addEventListener('devicemotion', e=>{
      const ax = e.acceleration?.x?.toFixed(2) ?? 0;
      const ay = e.acceleration?.y?.toFixed(2) ?? 0;
      const az = e.acceleration?.z?.toFixed(2) ?? 0;
      set("acceleration", `Accélération : x=${ax} y=${ay} z=${az} (IMU)`);
    });
  }

  // Lumière
  if('AmbientLightSensor' in window){
    const sensor = new AmbientLightSensor();
    sensor.addEventListener('reading', ()=>set("lumiere", `Luminosité : ${sensor.illuminance} lx (Capteur lumière)`));
    sensor.start();
  }
}

// Stop GPS et capteurs
export function stop() {
  if(watchId!==null){
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
  }
}

// Reset
export function resetAll() {
  stop();
  prevPos=null; distanceTotale=0; vitesses=[];
  const ids = ["gps","latitude","longitude","altitude","vitesse","vitesse-moy","distance","orientation","acceleration","lumiere"];
  for(const id of ids) set(id, `${id} : --`);
}

// Callback GPS
function onPosition(pos){
  const c = pos.coords;
  set("gps", `GPS précision : ${c.accuracy.toFixed(1)} m`);
  set("latitude", `Latitude : ${c.latitude.toFixed(6)} (GPS)`);
  set("longitude", `Longitude : ${c.longitude.toFixed(6)} (GPS)`);
  set("altitude", `Altitude : ${c.altitude?.toFixed(1) ?? "--"} m (GPS + Baromètre)`);

  if(prevPos){
    const dt = Math.max((pos.timestamp-prevPos.timestamp)/1000,0.001);
    const d = calculerDistance(prevPos,c);
    distanceTotale += d;
    const v = (d/dt)*3.6;
    vitesses.push(v);
    if(vitesses.length>1000) vitesses.shift();
    const moy = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;

    set("vitesse", `Vitesse : ${v.toFixed(2)} km/h (GPS + IMU)`);
    set("vitesse-moy", `Vitesse moyenne : ${moy.toFixed(2)} km/h`);
    set("distance", `Distance : ${distanceTotale.toFixed(1)} m`);
  }

  prevPos = {...c, timestamp: pos.timestamp};
}

function onError(err){
  set("gps", `Erreur GPS : ${err.message}`);
}

// Calcul distance en mètres (Haversine)
function calculerDistance(a,b){
  const R = 6371e3;
  const φ1 = a.latitude*Math.PI/180;
  const φ2 = b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180;
  const Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const s = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return 2*R*Math.atan2(Math.sqrt(s),Math.sqrt(1-s));
        }
                            
