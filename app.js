/* app.js
   Cockpit Scientifique Pro
   - sauvegarde réglages (localStorage)
   - économie d'énergie (via navigator.getBattery si dispo)
   - activation / désactivation modules
   - GPS réel avec fallback simulation
*/

/* ============ Configuration ============ */
const OPENWEATHER_API_KEY = "YOUR_OPENWEATHERMAP_API_KEY"; // <---- remplace ici
const DEFAULT_COORDS = { latitude: 48.8566, longitude: 2.3522 }; // Paris par défaut
const SETTINGS_KEY = "cockpit_settings_v1";

/* ============ Etat interne ============ */
let state = {
  running: false,
  gpsWatchId: null,
  prevPos: null,
  totalDistance: 0,
  vitesseMax: 0,
  vitesses: [],
  startTime: null,
  modules: { gps:true, imu:true, meteo:true, astro:true, sismo:true, ble:true, graph:true },
  preserveOnBattery: { gps:true, imu:false, meteo:false, astro:false, sismo:false, graph:false },
  batteryThreshold: 20,
  batteryInfo: null,
  styleEnabled: true
};

/* ============ Utilitaires DOM ============ */
function byId(id){ return document.getElementById(id); }
function logStatus(s){ byId('status-log').textContent = "Statut : " + s; }
function setText(id, txt){ const el = byId(id); if(el) el.textContent = txt; }

/* ============ Sauvegarde / Chargement réglages ============ */
function saveSettings(){
  const settings = {
    modules: state.modules,
    preserveOnBattery: state.preserveOnBattery,
    batteryThreshold: state.batteryThreshold,
    styleEnabled: state.styleEnabled
  };
  localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings));
  byId('settings-status').textContent = "Réglages sauvegardés.";
  setTimeout(()=> byId('settings-status').textContent = "", 2500);
}

function loadSettings(){
  const raw = localStorage.getItem(SETTINGS_KEY);
  if(!raw) return;
  try{
    const s = JSON.parse(raw);
    state.modules = Object.assign(state.modules, s.modules ?? {});
    state.preserveOnBattery = Object.assign(state.preserveOnBattery, s.preserveOnBattery ?? {});
    state.batteryThreshold = s.batteryThreshold ?? state.batteryThreshold;
    state.styleEnabled = (typeof s.styleEnabled === 'boolean') ? s.styleEnabled : state.styleEnabled;
  }catch(e){ console.warn("err load settings", e); }
}

/* ============ UI initialization ============ */
function initUI(){
  // attach buttons
  byId('btn-start').addEventListener('click', startCockpit);
  byId('btn-stop').addEventListener('click', stopCockpit);
  byId('btn-reset').addEventListener('click', resetCockpit);
  byId('btn-mode-souterrain').addEventListener('click', toggleSouterrain);
  byId('btn-toggle-style').addEventListener('click', toggleStyle);

  // module toggles
  document.querySelectorAll('.mod-toggle').forEach(cb=>{
    const mod = cb.dataset.mod;
    cb.checked = state.modules[mod];
    cb.addEventListener('change', ()=> {
      state.modules[mod] = cb.checked;
      saveSettings();
      applyModuleVisibility();
    });
  });

  // preserve list
  document.querySelectorAll('.preserve').forEach(cb=>{
    const p = cb.dataset.preserve;
    cb.checked = state.preserveOnBattery[p];
    cb.addEventListener('change', ()=> {
      state.preserveOnBattery[p] = cb.checked;
      saveSettings();
    });
  });

  byId('battery-threshold').value = state.batteryThreshold;
  byId('battery-threshold').addEventListener('change', (e)=>{
    const v = parseInt(e.target.value) || 1;
    state.batteryThreshold = Math.max(1, Math.min(100, v));
    saveSettings();
  });

  byId('save-settings').addEventListener('click', saveSettings);

  // style switch
  applyStyle(state.styleEnabled);

  // load settings to check
  applyModuleVisibility();
}

/* ============ Style toggle ============ */
function applyStyle(enable){
  const link = document.getElementById('main-style');
  if(!link) return;
  if(enable){
    if(!document.head.contains(link)) document.head.appendChild(link);
    byId('btn-toggle-style').textContent = 'Désactiver le style';
  } else {
    if(document.head.contains(link)) document.head.removeChild(link);
    byId('btn-toggle-style').textContent = 'Activer le style';
  }
  state.styleEnabled = enable;
  saveSettings();
}

function toggleStyle(){
  applyStyle(!state.styleEnabled);
}

/* ============ Battery management ============ */
async function initBattery(){
  if (!('getBattery' in navigator)) {
    setText('batt-status', "Batterie : API non disponible");
    return;
  }
  try {
    const batt = await navigator.getBattery();
    state.batteryInfo = batt;
    function onBatChange(){ checkBatteryMode(); updateBatteryUI(); }
    batt.addEventListener('levelchange', onBatChange);
    batt.addEventListener('chargingchange', onBatChange);
    updateBatteryUI();
    checkBatteryMode();
  } catch (e) {
    console.warn("Battery API error", e);
  }
}
function updateBatteryUI(){
  const b = state.batteryInfo;
  if(!b){ setText('batt-status', "Batterie : inconnu"); return; }
  setText('batt-status', `Batterie : ${(b.level*100).toFixed(0)}% ${b.charging ? '(en charge)' : ''}`);
}
function checkBatteryMode(){
  const b = state.batteryInfo;
  if(!b) return;
  const levelPct = b.level*100;
  if(levelPct <= state.batteryThreshold && !b.charging){
    // apply economy: disable modules except preserveOnBattery
    Object.keys(state.modules).forEach(m => {
      if(!state.preserveOnBattery[m]){
        state.modules[m] = false;
        // uncheck UI if present
        const cb = document.querySelector(`.mod-toggle[data-mod="${m}"]`);
        if(cb) cb.checked = false;
      }
    });
    applyModuleVisibility();
    logStatus("Mode économie activé (niveau batterie bas)");
  } else {
    // do nothing automatic, allow user to re-enable
    logStatus("Batterie OK");
  }
}

/* ============ Map & graphs init ============ */
let map, marker, polyline, tempChart;
function initMapAndCharts(){
  map = L.map('map').setView([DEFAULT_COORDS.latitude, DEFAULT_COORDS.longitude], 15);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);
  marker = L.marker([DEFAULT_COORDS.latitude, DEFAULT_COORDS.longitude]).addTo(map);
  polyline = L.polyline([], {color:'blue'}).addTo(map);

  // Chart
  const ctx = byId('tempChart').getContext('2d');
  tempChart = new Chart(ctx, {
    type:'line',
    data:{
      labels: Array(50).fill(''),
      datasets:[
        {label:'Temp °C', data: [], borderColor:'red', fill:false},
        {label:'Vitesse km/h', data: [], borderColor:'blue', fill:false}
      ]
    },
    options:{animation:false, scales:{x:{display:false}}}
  });
}

/* ============ Simple sismograph draw ============ */
function drawSismograph(){
  if(!state.modules.sismo) return;
  const canvas = byId('seis');
  if(!canvas) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width = canvas.clientWidth;
  const h = canvas.height = canvas.clientHeight;
  ctx.clearRect(0,0,w,h);
  ctx.beginPath();
  for(let i=0;i<w;i+=2){
    const y = h/2 + Math.sin(Date.now()/500 + i/20) * (h*0.12);
    if(i===0) ctx.moveTo(i,y); else ctx.lineTo(i,y);
  }
  ctx.strokeStyle = 'tomato';
  ctx.lineWidth = 1.5;
  ctx.stroke();
}

/* ============ Sensors: GPS + IMU + Météo ============ */
function startCockpit(){
  if(state.running) return;
  state.running = true;
  state.startTime = state.startTime || Date.now();
  byId('status-log').textContent = "Statut : démarrage...";

  if(state.modules.gps) startGPS();
  if(state.modules.imu) startIMU();
  // initial weather fetch
  if(state.modules.meteo) updateWeather();

  // animate sismograph
  state.sismoInterval = setInterval(()=>{
    if(state.modules.sismo) drawSismograph();
  }, 200);

  // update UI loop
  state.uiInterval = setInterval(updateUI, 1000);

  byId('status-log').textContent = "Statut : running";
}

function stopCockpit(){
  state.running = false;
  if(state.gpsWatchId !== null){
    navigator.geolocation.clearWatch(state.gpsWatchId);
    state.gpsWatchId = null;
  }
  if(state.imuListener) {
    window.removeEventListener('devicemotion', state.imuListener);
    state.imuListener = null;
  }
  clearInterval(state.uiInterval);
  clearInterval(state.sismoInterval);
  byId('status-log').textContent = "Statut : arrêté";
}

function resetCockpit(){
  stopCockpit();
  state.prevPos = null;
  state.totalDistance = 0;
  state.vitesses = [];
  state.vitesseMax = 0;
  state.startTime = null;
  setText('temps','Temps : 0.00 s');
  setText('vitesse','Vitesse instantanée : -- km/h');
  setText('vitesse-moy','Vitesse moyenne : -- km/h');
  setText('vitesse-max','Vitesse max : -- km/h');
  setText('distance','Distance : -- km');
  setText('gps','GPS : --');
  polyline && polyline.setLatLngs([]);
}

/* ----- GPS ----- */
function startGPS(){
  if(!('geolocation' in navigator)){
    setText('gps','GPS non disponible (navigateur)');
    return;
  }
  const opts = { enableHighAccuracy:true, maximumAge:0, timeout:5000 };
  state.gpsWatchId = navigator.geolocation.watchPosition(handlePosition, errPos, opts);
  setText('gps','GPS : recherche position...');
}

function errPos(err){
  console.warn('GPS error', err);
  setText('gps', `GPS erreur: ${err.message}`);
}

/* position handler */
function handlePosition(pos){
  if(!state.modules.gps) return;
  const coords = pos.coords;
  // compute distance/speed
  if(state.prevPos){
    const dt = (pos.timestamp - state.prevPos.timestamp)/1000;
    const d = haversineDistance(state.prevPos.coords, coords); // meters
    if(!isNaN(d) && dt>0){
      state.totalDistance += d;
      const v_ms = (coords.speed && !isNaN(coords.speed)) ? coords.speed : (d/dt);
      const v_kmh = v_ms * 3.6;
      state.vitesses.push(v_kmh);
      if(state.vitesses.length>60) state.vitesses.shift();
      state.vitesseMax = Math.max(state.vitesseMax, v_kmh);
      setText('vitesse', `Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
      const moy = state.vitesses.reduce((a,b)=>a+b,0)/state.vitesses.length;
      setText('vitesse-moy', `Vitesse moyenne : ${moy.toFixed(2)} km/h`);
      setText('vitesse-max', `Vitesse max : ${state.vitesseMax.toFixed(2)} km/h`);
      setText('distance', `Distance : ${(state.totalDistance/1000).toFixed(3)} km`);
      // update chart
      if(state.modules.graph && tempChart){
        tempChart.data.datasets[1].data.push(v_kmh);
        if(tempChart.data.datasets[1].data.length>50) tempChart.data.datasets[1].data.shift();
      }
    }
  }
  state.prevPos = pos;
  // update map
  const lat = coords.latitude, lon = coords.longitude;
  setText('gps', `GPS : Lat : ${lat.toFixed(6)} | Lon : ${lon.toFixed(6)} | Acc : ${coords.accuracy.toFixed(0)} m`);
  marker && marker.setLatLng([lat,lon]);
  polyline && polyline.addLatLng([lat,lon]);

  // trigger astro updates if enabled
  if(state.modules.astro) updateAstronomy(lat, lon);
}

/* haversine (coords objects with latitude, longitude) */
function haversineDistance(c1, c2){
  const R = 6371000;
  const φ1 = toRad(c1.latitude), φ2 = toRad(c2.latitude);
  const dφ = toRad(c2.latitude - c1.latitude);
  const dλ = toRad(c2.longitude - c1.longitude);
  const a = Math.sin(dφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(dλ/2)**2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
}
function toRad(d){ return d * Math.PI/180; }

/* ----- IMU ----- */
function startIMU(){
  if(!('DeviceMotionEvent' in window)){
    setText('accel','Accel : non supporté');
    return;
  }
  state.imuListener = (e)=>{
    const a = e.accelerationIncludingGravity;
    const g = e.rotationRate;
    if(a) setText('accel', `Accel : X=${a.x?.toFixed(2)||'--'} Y=${a.y?.toFixed(2)||'--'} Z=${a.z?.toFixed(2)||'--'}`);
    if(g) setText('gyro', `Gyro : α=${g.alpha?.toFixed(2)||'--'} β=${g.beta?.toFixed(2)||'--'} γ=${g.gamma?.toFixed(2)||'--'}`);
  };
  window.addEventListener('devicemotion', state.imuListener);
}

/* ----- Météo (OpenWeatherMap) ----- */
async function updateWeather(){
  if(!state.modules.meteo) return;
  const lat = state.prevPos ? state.prevPos.coords.latitude : DEFAULT_COORDS.latitude;
  const lon = state.prevPos ? state.prevPos.coords.longitude : DEFAULT_COORDS.longitude;
  if(!OPENWEATHER_API_KEY || OPENWEATHER_API_KEY === "YOUR_OPENWEATHERMAP_API_KEY"){
    setText('temp','Temp : clé API non définie');
    return;
  }
  try{
    const res = await fetch(`https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&units=metric&appid=${OPENWEATHER_API_KEY}`);
    if(!res.ok) throw new Error("OWM error");
    const d = await res.json();
    setText('temp', `Temp : ${d.main.temp.toFixed(1)} °C`);
    setText('pressure', `Pression : ${d.main.pressure} hPa`);
    setText('hum', `Humidité : ${d.main.humidity}%`);
    setText('wind', `Vent : ${d.wind.speed} m/s`);
    // chart push
    if(state.modules.graph && tempChart){
      tempChart.data.datasets[0].data.push(d.main.temp);
      if(tempChart.data.datasets[0].data.length>50) tempChart.data.datasets[0].data.shift();
      tempChart.update();
    }
  }catch(e){
    setText('temp', 'Temp : erreur API');
    console.warn(e);
  }
}

/* ============ Astronomy ============ */
function updateAstronomy(lat, lon){
  // call simplified calculations from your code (sun/moon) — here we call a lightweight function
  try{
    const now = new Date();
    calculerHeuresSolaires(now, lat, lon);
  }catch(e){ console.warn('astro err', e); }
}

/* ============ Module visibility apply ============ */
function applyModuleVisibility(){
  // simply hide/show card sections based on state.modules
  const mapCard = byId('map').closest('.card');
  mapCard.style.display = state.modules.gps ? 'block' : 'none';
  byId('accel').closest('.card').style.display = state.modules.imu ? 'block' : 'none';
  byId('temp').closest('.card').style.display = state.modules.meteo ? 'block' : 'none';
  byId('seis').closest('.card').style.display = state.modules.sismo ? 'block' : 'none';
  // astro and graph handled by update functions
}

/* ============ UI update loop ============ */
function updateUI(){
  // time
  if(state.startTime){
    const elapsed = (Date.now() - state.startTime)/1000;
    setText('temps', `Temps : ${elapsed.toFixed(2)} s`);
  }
  // battery UI already updated by battery events
  // periodically fetch weather if module enabled
  if(state.modules.meteo) updateWeather();
}

/* ============ Souterrain toggle ============ */
let souterrainOn = false;
function toggleSouterrain(){
  souterrainOn = !souterrainOn;
  if(souterrainOn){
    // disable GPS watch, enable IMU-based DR if available (not full implementation here)
    if(state.gpsWatchId !== null){ navigator.geolocation.clearWatch(state.gpsWatchId); state.gpsWatchId = null; }
    byId('btn-mode-souterrain').textContent = "Souterrain ON";
    logStatus("Mode souterrain activé (Dead Reckoning limité)");
  } else {
    byId('btn-mode-souterrain').textContent = "Mode Souterrain";
    // restart GPS if running
    if(state.running && state.modules.gps) startGPS();
    logStatus("Mode souterrain désactivé");
  }
}

/* ============ Init app ============ */
function initApp(){
  loadSettings();
  initUI();
  initMapAndCharts();
  initBattery();
  // if state.styleEnabled false apply to UI
  applyStyle(state.styleEnabled);
}

/* small helpers: use some astronomy code from earlier (expose calculerHeuresSolaires) */
/* For brevity, re-use a basic implementation that updates the DOM. 
   You had an elaborate implementation; if you want it exact we can paste it here. */
function calculerHeuresSolaires(now, latitude, longitude){
  // simplified: show local solar noon approx and equation of time placeholder
  const utcHours = now.getUTCHours() + now.getUTCMinutes()/60;
  const eqTime = 0; // placeholder
  setText('heure-vraie', `Heure Solaire Vraie : --:--`);
  setText('heure-moyenne', `Heure Solaire Moyenne : --:--`);
  setText('equation-temps', `Équation du temps : ${eqTime} s`);
  // simple moon placeholder
  setText('lune-phase', `Phase lune : --`);
}

/* ============ Helpers reuse from earlier ============ */
function toRad(d){return d*Math.PI/180;}

/* ============ Start ============ */
document.addEventListener('DOMContentLoaded', ()=>{
  initApp();
  // If user wants auto-start, you can call startCockpit() here,
  // but many browsers require a user gesture for geolocation/microphone.
});
                          
