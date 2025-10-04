let watchId = null;
let prevPos = null;
let distanceTotale = 0;
let vitesses = [];

function set(id, txt) {
  const el = document.getElementById(id);
  if(el) el.textContent = txt;
}

function demarrer() {
  if(!navigator.geolocation) return set("gps","Géolocalisation non disponible");
  if(watchId!==null) return set("gps","GPS déjà actif");

  distanceTotale = 0; vitesses = []; prevPos = null;

  watchId = navigator.geolocation.watchPosition(onPosition, onGeoError, {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 20000
  });
  set("gps","Recherche position...");
}

function stop(){
  if(watchId!==null){
    navigator.geolocation.clearWatch(watchId);
    watchId=null;
    set("gps","GPS arrêté");
  }
}

function resetAll(){
  stop();
  prevPos=null; distanceTotale=0; vitesses=[];
  set("vitesse","Vitesse : -- km/h");
  set("vitesse-moy","Vitesse moyenne : -- km/h");
  set("vitesse-ms","Vitesse : -- m/s");
  set("distance","Distance : -- m");
  set("gps","GPS : --");
}

function onPosition(pos){
  const c=pos.coords;
  set("gps",`GPS précision : ${c.accuracy.toFixed(1)} m`);

  if(c.accuracy > 50) return; // ignore positions imprécises

  if(prevPos){
    const dt = Math.max((pos.timestamp-prevPos.timestamp)/1000,0.001);
    const d = calculerDistance(prevPos,c);
    distanceTotale += d;

    const v = (d/dt)*3.6;
    if(!isNaN(v)){
      vitesses.push(v);
      if(vitesses.length>1000) vitesses.shift();
      const moy = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;
      const mps = v/3.6;

      set("vitesse",`Vitesse : ${v.toFixed(1)} km/h`);
      set("vitesse-moy",`Vitesse moyenne : ${moy.toFixed(1)} km/h`);
      set("vitesse-ms",`Vitesse : ${mps.toFixed(2)} m/s`);
      set("distance",`Distance : ${distanceTotale.toFixed(1)} m`);
    }
  }

  prevPos = {...c,timestamp:pos.timestamp};
}

function onGeoError(err){
  set("gps",`Erreur GPS : ${err.message}`);
}

function calculerDistance(a,b){
  const R = 6371e3;
  const φ1 = a.latitude*Math.PI/180;
  const φ2 = b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180;
  const Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const s = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return 2*R*Math.atan2(Math.sqrt(s),Math.sqrt(1-s));
                        }
