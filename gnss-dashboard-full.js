/* app.js — Partie 1/5
   GNSS SpaceTime Dashboard — Fusion complète
   Partie 1 : constantes, utilitaires, storage, conversions, physics helpers
*/

/* -------------------- UTIL -------------------- */
const $ = id => document.getElementById(id);
const nowISO = () => new Date().toISOString();
function safeSet(id, txt){ const e=$(id); if(e) e.textContent = txt; }

/* -------------------- BACKOFF FETCH -------------------- */
async function fetchWithBackoff(url, opts={}, retries=3, delay=600){
  for(let i=0;i<=retries;i++){
    try{
      const res = await fetch(url, opts);
      if(!res.ok) throw new Error('HTTP '+res.status);
      return await res.json();
    }catch(e){
      if(i===retries) { console.warn('fetch fail', url, e); throw e; }
      await new Promise(r=>setTimeout(r, delay));
      delay *= 2;
    }
  }
}

/* -------------------- STORAGE (IndexedDB minimal) -------------------- */
const Storage = (function(){
  const DB_NAME='gnss_dash_db', STORE='observations';
  let db=null;
  function openDB(){
    return new Promise((res, rej) => {
      if(db) return res(db);
      const r=indexedDB.open(DB_NAME,1);
      r.onupgradeneeded = e => { const d=e.target.result; if(!d.objectStoreNames.contains(STORE)) d.createObjectStore(STORE, {keyPath:'ts'}); };
      r.onsuccess = e => { db = e.target.result; res(db); };
      r.onerror = e => rej(e);
    });
  }
  async function save(obj){
    const d = await openDB();
    return new Promise((res, rej) => {
      const tx = d.transaction(STORE,'readwrite'); const s = tx.objectStore(STORE);
      s.put(obj);
      tx.oncomplete = ()=> res(true);
      tx.onerror = e => rej(e);
    });
  }
  async function all(){
    const d = await openDB();
    return new Promise((res, rej) => {
      const tx = d.transaction(STORE,'readonly'); const s = tx.objectStore(STORE);
      const out=[]; const cur = s.openCursor();
      cur.onsuccess = e => { const c = e.target.result; if(c){ out.push(c.value); c.continue(); } else res(out); };
      cur.onerror = e => rej(e);
    });
  }
  async function clear(){ const d = await openDB(); return new Promise((res, rej)=>{ const tx=d.transaction(STORE,'readwrite'); tx.objectStore(STORE).clear(); tx.oncomplete=()=>res(true); tx.onerror=e=>rej(e); }); }
  return {save, all, clear};
})();

/* -------------------- UNITS & CONVERSIONS -------------------- */
const Units = {
  m_to_km: m => m/1000,
  km_to_m: km => km*1000,
  m_to_ft: m => m*3.280839895,
  ft_to_m: ft => ft/3.280839895,
  ms_to_kmh: v => v*3.6,
  kmh_to_ms: v => v/3.6,
  c_to_k: c => c+273.15,
  k_to_c: k => k-273.15,
  hpa_to_pa: p => p*100,
  pa_to_hpa: p => p/100,
  frac_of_c: v => v/299792458,
  lorentz_gamma: v => 1/Math.sqrt(1-Math.pow(v/299792458,2)),
  mach: (v,c)=> v/c
};

/* -------------------- PHYSICS HELPERS -------------------- */
const Physics = (function(){
  const gamma_air = 1.4;
  function satVap_hPa(T_C){
    const a=17.27, b=237.7;
    return 6.1078 * Math.exp((a*T_C)/(b+T_C));
  }
  function dewPoint(T_C, RH_pct){
    const a=17.27,b=237.7;
    const alpha = ((a*T_C)/(b+T_C)) + Math.log(RH_pct/100);
    return (b*alpha)/(a-alpha);
  }
  function densityAir(T_C, P_hPa, RH_pct=50){
    const T_K = T_C + 273.15;
    const es = satVap_hPa(T_C);
    const e = (RH_pct/100)*es;
    const x_v = (P_hPa>0)? (e / P_hPa) : 0;
    const r_spec_dry = 287.058;
    const r_spec_water = 461.495;
    const Rmix = (1-x_v)*r_spec_dry + x_v*r_spec_water;
    return (P_hPa*100) / (Rmix * T_K);
  }
  function speedOfSound(T_C, RH_pct=50, P_hPa=1013.25){
    const T_K = T_C + 273.15;
    const es = satVap_hPa(T_C);
    const e = (RH_pct/100)*es;
    const x_v = (P_hPa>0)? (e/P_hPa) : 0;
    const r_spec_dry = 287.058;
    const r_spec_water = 461.495;
    const Rmix = (1-x_v)*r_spec_dry + x_v*r_spec_water;
    return Math.sqrt(gamma_air * Rmix * T_K);
  }
  function sutherlandViscosity(T_C){
    const T = T_C + 273.15;
    const mu0 = 1.716e-5, T0=273.15, C=110.4;
    return mu0 * ((T0+C)/(T+C)) * Math.pow(T/T0, 1.5);
  }
  function barometricAltitude(P_hPa, P0_hPa=1013.25, T0_C=15){
    const T0 = T0_C + 273.15;
    const g0 = 9.80665;
    const L = 0.0065;
    const R = 287.05;
    const h = (T0 / L) * (1 - Math.pow(P_hPa / P0_hPa, (R*L)/g0));
    return h;
  }
  function somigliana(lat_deg){
    const phi = lat_deg * Math.PI/180;
    const g0 = 9.7803253359;
    const k = 0.00193185265241;
    const e2 = 0.00669437999013;
    return g0 * (1 + k * Math.pow(Math.sin(phi),2)) / Math.sqrt(1 - e2 * Math.pow(Math.sin(phi),2));
  }
  // ppm -> µg/m3 generic (requires molar mass in g/mol)
  function ppm_to_ugm3(ppm, M_gpermol, T_K=293.15, P_Pa=101325){
    const R = 8.314462618;
    const mol_per_m3 = P_Pa / (R * T_K);
    const g_m3 = mol_per_m3 * M_gpermol * (ppm * 1e-6);
    return g_m3 * 1e6;
  }
  // AQI US EPA mapping (PM2.5)
  function calcAqiFromPm25(pm25){
    const bp=[{Clow:0,Chigh:12,Ilow:0,Ihigh:50},{Clow:12.1,Chigh:35.4,Ilow:51,Ihigh:100},{Clow:35.5,Chigh:55.4,Ilow:101,Ihigh:150},{Clow:55.5,Chigh:150.4,Ilow:151,Ihigh:200},{Clow:150.5,Chigh:250.4,Ilow:201,Ihigh:300},{Clow:250.5,Chigh:350.4,Ilow:301,Ihigh:400},{Clow:350.5,Chigh:500,Ilow:401,Ihigh:500}];
    for(const b of bp){ if(pm25>=b.Clow && pm25<=b.Chigh){ return Math.round(((b.Ihigh-b.Ilow)/(b.Chigh-b.Clow))*(pm25-b.Clow)+b.Ilow); } }
    return null;
  }
  return {dewPoint, densityAir, speedOfSound, sutherlandViscosity, barometricAltitude, somigliana, ppm_to_ugm3, calcAqiFromPm25};
})();
          /* app.js — Partie 2/5
   Map, charts, UI wiring, API wrappers (Open-Meteo, AirQuality, Open-Notify proxy)
*/

/* -------------------- MAP & VISUALS -------------------- */
let map, userMarker, issMarker, issTrail;
try{
  map = L.map('map', {attributionControl:false}).setView([48.8566,2.3522],5);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);
  userMarker = L.marker([48.8566,2.3522]).addTo(map).bindPopup('Vous');
  issMarker = L.marker([0,0], {icon: L.divIcon({className:'iss-icon', html:'🛰️', iconSize:[24,24]})}).addTo(map);
  issTrail = L.polyline([], {color:'cyan'}).addTo(map);
}catch(e){ console.warn('leaflet not ready', e); }

/* -------------------- CHARTS -------------------- */
let chartTemp = null, chartAQI = null;
try{
  const ctxT = $('chartTemp').getContext('2d');
  chartTemp = new Chart(ctxT, { type:'line', data:{labels:[], datasets:[{label:'Temp (°C)', data:[], tension:0.3}]}, options:{plugins:{legend:{display:false}}, scales:{x:{display:false}}} });
  const ctxA = $('chartAQI').getContext('2d');
  chartAQI = new Chart(ctxA, { type:'bar', data:{labels:['PM2.5','PM10','O3','NO2'], datasets:[{label:'µg/m³', data:[0,0,0,0]}]}, options:{plugins:{legend:{display:false}}} });
}catch(e){ console.warn('charts init fail', e); }

/* -------------------- API WRAPPERS -------------------- */
const API = (function(){
  async function weather(lat, lon){
    const url = `https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}&current_weather=true&hourly=temperature_2m,relativehumidity_2m,dewpoint_2m,pressure_msl,uv_index&timezone=auto`;
    return await fetchWithBackoff(url);
  }
  async function air(lat, lon){
    const url = `https://air-quality-api.open-meteo.com/v1/air-quality?latitude=${lat}&longitude=${lon}&hourly=pm2_5,pm10,o3,no2`;
    return await fetchWithBackoff(url);
  }
  async function iss(){
    // open-notify HTTP; prefer proxy (allorigins) to avoid mixed content
    const proxy = 'https://api.allorigins.win/raw?url=';
    const url = proxy + encodeURIComponent('http://api.open-notify.org/iss-now.json');
    return await fetchWithBackoff(url);
  }
  return {weather, air, iss};
})();

/* -------------------- UI HOOKS -------------------- */
$('btn-locate')?.addEventListener('click', ()=> {
  if(!navigator.geolocation) return alert('Géo non supportée');
  navigator.geolocation.getCurrentPosition(async p=>{
    const lat=p.coords.latitude, lon=p.coords.longitude, alt=p.coords.altitude||0, acc=p.coords.accuracy||0;
    safeSet('lat', lat.toFixed(6)); safeSet('lon', lon.toFixed(6)); safeSet('alt', (alt||0).toFixed(2)+' m');
    safeSet('gps-acc', acc? acc.toFixed(1)+' m' : 'N/A');
    userMarker?.setLatLng([lat,lon]); map?.setView([lat,lon], 12);
    await orchestrator.runRefresh(lat, lon);
  }, e=> alert('GEO error: '+ e.message), {enableHighAccuracy:true});
});

$('btn-watch')?.addEventListener('click', ()=> startWatch());
$('btn-stop-watch')?.addEventListener('click', ()=> stopWatch());
$('btn-imu')?.addEventListener('click', ()=> startImu());
$('btn-export-csv')?.addEventListener('click', ()=> exportObservationsCSV());
$('btn-clear-logs')?.addEventListener('click', async ()=> { await Storage.clear(); alert('Logs cleared'); });

$('btn-iss-track')?.addEventListener('click', ()=> { followIss = !followIss; $('btn-iss-track').textContent = followIss? '🛰 Suivi ON' : '🛰 Suivre ISS'; });
$('btn-show-trail')?.addEventListener('click', ()=> { issTrailOn = !issTrailOn; issTrail?.setStyle({opacity: issTrailOn?1:0}); });
$('btn-replay')?.addEventListener('click', ()=> replayTrail());

/* -------------------- Export CSV -------------------- */
async function exportObservationsCSV(){
  const rows = await Storage.all();
  if(!rows || rows.length===0) return alert('Aucun enregistrement');
  const keys = Object.keys(rows[0]);
  const csv = [keys.join(',')].concat(rows.map(r=> keys.map(k=>JSON.stringify(r[k]||'')).join(','))).join('\n');
  const blob = new Blob([csv], {type:'text/csv'}); const url = URL.createObjectURL(blob);
  const a = document.createElement('a'); a.href = url; a.download = 'observations.csv'; a.click(); URL.revokeObjectURL(url);
              }

/* app.js — Partie 3/5
   Sensors: GPS watch, IMU, other sensors; EKF/UKF plugin architecture
*/

/* -------------------- SENSORS & WATCH -------------------- */
let geoWatchId = null;
function startWatch(){
  if(!navigator.geolocation) return alert('Géo non supportée');
  geoWatchId = navigator.geolocation.watchPosition(async p=>{
    const lat=p.coords.latitude, lon=p.coords.longitude, alt=p.coords.altitude||0, acc=p.coords.accuracy||0, speed=p.coords.speed||0;
    safeSet('lat', lat.toFixed(6)); safeSet('lon', lon.toFixed(6)); safeSet('alt', (alt||0).toFixed(2)+' m'); safeSet('gps-acc', acc? acc.toFixed(1)+' m' : 'N/A');
    safeSet('speed-raw-ms', speed? speed.toFixed(3): 'N/A');
    // EKF/UKF update
    if(!filterEngine.initialized) filterEngine.initFromGnss(lat,lon,alt);
    const dt = (Date.now() - (filterEngine.lastTs||Date.now()))/1000;
    filterEngine.predict(dt);
    filterEngine.updateGnss(lat, lon, alt, Math.max(3, acc||5));
    filterEngine.lastTs = Date.now();
    // persist
    await Storage.save({ts: nowISO(), source:'gps', lat, lon, alt, acc, speed});
    // refresh derived & UI
    await orchestrator.runRefresh(lat, lon);
  }, e=> console.warn('watch err', e), {enableHighAccuracy:true, maximumAge:1000, timeout:10000});
  safeSet('gps-status','watching');
}

function stopWatch(){
  if(geoWatchId!==null){ navigator.geolocation.clearWatch(geoWatchId); geoWatchId=null; safeSet('gps-status','stopped'); }
}

/* -------------------- IMU (DeviceMotion) -------------------- */
let imuActive=false;
function startImu(){
  if(typeof DeviceMotionEvent === 'undefined'){ safeSet('imu-status','N/A'); return alert('IMU non supportée'); }
  const onMotion = (ev) => {
    const acc = ev.acceleration || ev.accelerationIncludingGravity || {x:0,y:0,z:0};
    safeSet('accel-x', (acc.x||0).toFixed(3)); safeSet('accel-y', (acc.y||0).toFixed(3)); safeSet('accel-z', (acc.z||0).toFixed(3));
    // integrate IMU into filter (best-effort)
    const dt = (Date.now() - (filterEngine.lastTs||Date.now()))/1000;
    if(dt>0 && filterEngine.updateImu) filterEngine.updateImu([acc.x||0, acc.y||0, acc.z||0], dt);
    filterEngine.lastTs = Date.now();
    Storage.save({ts:nowISO(), source:'imu', acc:{x:acc.x,y:acc.y,z:acc.z}});
  };
  if(typeof DeviceMotionEvent.requestPermission === 'function'){
    DeviceMotionEvent.requestPermission().then(res=>{ if(res==='granted'){ window.addEventListener('devicemotion', onMotion); imuActive=true; safeSet('imu-status','Actif'); } else safeSet('imu-status','Refusé'); }).catch(e=>{ console.warn(e); safeSet('imu-status','Erreur'); });
  } else { window.addEventListener('devicemotion', onMotion); imuActive=true; safeSet('imu-status','Actif'); }
}

/* -------------------- Sensor samples: Magnetometer & Barometer (if available) -------------------- */
if('AbsoluteOrientationSensor' in window || 'Magnetometer' in window){
  try{
    const mag = new (window.Magnetometer || window.AbsoluteOrientationSensor)();
    mag.addEventListener('reading', ()=> {
      // Many browsers require HTTPS and permissions; this is best-effort
      safeSet('mag-x', mag.x ? mag.x.toFixed(2) : 'N/A');
      safeSet('mag-y', mag.y ? mag.y.toFixed(2) : 'N/A');
      safeSet('mag-z', mag.z ? mag.z.toFixed(2) : 'N/A');
    });
    mag.start();
  }catch(e){ console.warn('mag start fail', e); }
}

/* -------------------- FILTER ENGINE (EKF/UKF plugin) -------------------- */
/* We include a robust EKF by default and allow loading UKF/other variants as plugins.
   The plugin system is simple: provide an object with init/predict/updateGnss/updateImu/getState
*/
const filterEngine = {
  type: 'EKF',
  instance: null,
  initialized: false,
  lastTs: null,
  // init boot
  initFromGnss(lat, lon, alt){
    if(!this.instance){ this.instance = new EKFDefault(); }
    this.instance.init(lat, lon, alt);
    this.initialized = true;
    this.lastTs = Date.now();
    safeSet('filter-status','initiated');
  },
  predict(dt){ if(this.instance && this.instance.predict) this.instance.predict(dt); },
  updateGnss(lat, lon, alt, sigma){ if(this.instance && this.instance.updateGnss) this.instance.updateGnss(lat, lon, alt, sigma); },
  updateImu(acc, dt){ if(this.instance && this.instance.updateImu) this.instance.updateImu(acc, dt); },
  getState(){ return this.instance && this.instance.getState ? this.instance.getState() : null; }
};

/* -------------------- Default EKF implementation (stable) -------------------- */
function EKFDefault(){
  // uses ENU approx, 6-state (x,y,z,vx,vy,vz)
  this.x = math.zeros(6,1);
  this.P = math.multiply(math.identity(6), 100);
  this.origin = null;
  this.lastTs = null;
}
EKFDefault.prototype.init = function(lat, lon, alt){
  this.origin = {lat0:lat, lon0:lon, alt0:alt||0};
  const [x,y] = EKFModule.geodeticToXY(lat, lon, lat, lon);
  this.x = math.matrix([[x],[y],[alt||0],[0],[0],[0]]);
  this.P = math.multiply(math.identity(6), 10);
  this.lastTs = Date.now();
};
EKFDefault.prototype.predict = function(dt){
  if(dt<=0) return;
  const F = math.identity(6).toArray();
  F[0][3]=dt; F[1][4]=dt; F[2][5]=dt;
  this.x = math.multiply(F, this.x);
  this.P = math.add(math.multiply(F, math.multiply(this.P, math.transpose(F))), math.multiply(math.identity(6), 0.1));
};
EKFDefault.prototype.updateGnss = function(lat, lon, alt, sigma=5){
  if(!this.origin){ this.init(lat, lon, alt); return; }
  const [mx, my] = EKFModule.geodeticToXY(this.origin.lat0, this.origin.lon0, lat, lon);
  const mz = alt || 0;
  const z = math.matrix([[mx],[my],[mz]]);
  const H = math.zeros(3,6); H.subset(math.index(0,0),1); H.subset(math.index(1,1),1); H.subset(math.index(2,2),1);
  const y = math.subtract(z, math.multiply(H, this.x));
  const R = math.multiply(math.identity(3), sigma);
  const S = math.add(math.multiply(H, math.multiply(this.P, math.transpose(H))), R);
  const K = math.multiply(math.multiply(this.P, math.transpose(H)), math.inv(S));
  this.x = math.add(this.x, math.multiply(K, y));
  const I = math.identity(6);
  this.P = math.multiply(math.subtract(I, math.multiply(K,H)), this.P);
};
EKFDefault.prototype.updateImu = function(acc, dt){
  if(dt<=0) return;
  const dv = math.matrix([[acc[0]*dt],[acc[1]*dt],[acc[2]*dt]]);
  const vel = math.subset(this.x, math.index([3,4,5],0));
  const newVel = math.add(vel, dv);
  this.x = math.subset(this.x, math.index([3,4,5],0), newVel);
  const dp = math.multiply(0.5*dt*dt, dv);
  const pos = math.subset(this.x, math.index([0,1,2],0));
  this.x = math.subset(this.x, math.index([0,1,2],0), math.add(pos, dp));
  this.P = math.add(this.P, math.multiply(math.identity(6), 0.05));
};
EKFDefault.prototype.getState = function(){
  if(!this.origin) return null;
  const x = this.x.subset(math.index(0,0)), y=this.x.subset(math.index(1,0)), z=this.x.subset(math.index(2,0));
  const [lat, lon] = EKFModule.xyToGeodetic(this.origin.lat0, this.origin.lon0, x, y);
  return {lat, lon, alt: z};
};

/* Note: if you want to use a UKF or another EKF variant from the ZIP,
   implement a plugin object with same API (init/predict/updateGnss/updateImu/getState)
   and assign filterEngine.instance = new YourPlugin(); filterEngine.type='UKF-variant';
*/

/* app.js — Partie 4/5
   Orchestration: refreshing weather/air, derived physics computations,
   AQI, Sun/Moon, ISS polling and trail, storage of observations
*/

let followIss = false, issTrailOn = false;

/* -------------------- orchestrator -------------------- */
const orchestrator = {
  async runRefresh(lat, lon){
    try{
      const w = await API.weather(lat, lon);
      if(w && w.current_weather){
        const T = w.current_weather.temperature;
        const P = (w.hourly && w.hourly.pressure_msl? w.hourly.pressure_msl[0]: null);
        const RH = (w.hourly && w.hourly.relativehumidity_2m? w.hourly.relativehumidity_2m[0] : null);
        safeSet('temp', T!==null? `${T.toFixed(2)} °C` : 'N/A');
        safeSet('pressure', P!==null? `${P.toFixed(1)} hPa` : 'N/A');
        safeSet('rh', RH!==null? `${RH.toFixed(1)} %` : 'N/A');
        if(chartTemp && T!==null){ chartTemp.data.labels.push(new Date().toLocaleTimeString()); chartTemp.data.datasets[0].data.push(T); if(chartTemp.data.labels.length>96){ chartTemp.data.labels.shift(); chartTemp.data.datasets[0].data.shift(); } chartTemp.update(); }
        await Storage.save({ts:nowISO(), source:'weather', lat, lon, T, P, RH});
      }
      const a = await API.air(lat, lon);
      if(a && a.hourly){
        const pm25 = a.hourly.pm2_5? a.hourly.pm2_5[0] : 0;
        const pm10 = a.hourly.pm10? a.hourly.pm10[0] : 0;
        const o3 = a.hourly.o3? a.hourly.o3[0] : 0;
        const no2 = a.hourly.no2? a.hourly.no2[0] : 0;
        if(chartAQI){ chartAQI.data.datasets[0].data = [pm25,pm10,o3,no2]; chartAQI.update(); }
        const aqi = Physics.calcAqiFromPm25(pm25);
        safeSet('weather-status', aqi? `AQI ${aqi}`: 'N/A');
        await Storage.save({ts:nowISO(), source:'air', lat, lon, pm25, pm10, o3, no2});
      }
      // derived physics
      refreshDerived(lat, lon);
      // sun & moon
      updateSunMoon(lat, lon);
    }catch(e){ console.warn('orchestrator.runRefresh failed', e); }
  }
};

/* -------------------- Derived physics computations -------------------- */
function refreshDerived(lat, lon){
  try{
    const Ttxt = $('temp')?.textContent || '15';
    const T = parseFloat((Ttxt+'').replace(/[^\d.-]/g,'')) || 15;
    const Ptxt = $('pressure')?.textContent || '1013';
    const P = parseFloat((Ptxt+'').replace(/[^\d.-]/g,'')) || 1013.25;
    const RHtxt = $('rh')?.textContent || '50';
    const RH = parseFloat((RHtxt+'').replace(/[^\d.-]/g,'')) || 50;
    // density & viscosity
    const rho = Physics.densityAir(T, P, RH);
    safeSet('rho', isFinite(rho)? `${rho.toFixed(4)} kg/m³` : 'N/A');
    safeSet('mu', Physics.sutherlandViscosity(T).toExponential(3)+' Pa·s');
    // speed of sound
    const c = Physics.speedOfSound(T, RH, P);
    safeSet('c_sound', isFinite(c)? `${c.toFixed(3)} m/s` : 'N/A');
    // dynamic pressure & drag example using v (from GNSS if available)
    const state = filterEngine.getState();
    let v = 10; // default demonstration speed m/s
    if(state && state.v) v = state.v;
    safeSet('speed-stable-ms', (v).toFixed(3));
    const q = 0.5 * rho * v * v; safeSet('qdyn', isFinite(q)? `${q.toFixed(2)} Pa`:'N/A');
    const area = parseFloat($('area')?.value) || 0.5;
    const Cd = parseFloat($('cd')?.value) || 1.0;
    const drag = q * Cd * area; safeSet('drag', isFinite(drag)? `${drag.toFixed(3)} N` : 'N/A');
    // coriolis & gravity
    const latVal = parseFloat($('lat')?.textContent) || lat || 0;
    const Fc = 2 * (parseFloat($('mass')?.value)||70) * v * 7.2921150e-5 * Math.sin(latVal*Math.PI/180);
    safeSet('coriolis', isFinite(Fc)? Fc.toExponential(3)+' N':'N/A');
    safeSet('g_local', Physics.somigliana(latVal).toFixed(6)+' m/s²');
    // Mach & Lorentz
    safeSet('mach-number', (c? (v/c).toFixed(6):'N/A'));
    safeSet('lorentz-factor', (Units.lorentz_gamma(v)).toFixed(9));
  }catch(e){ console.warn('refreshDerived fail', e); }
}

/* -------------------- Sun/Moon (SunCalc) -------------------- */
function updateSunMoon(lat, lon){
  try{
    const now = new Date();
    const sp = SunCalc.getPosition(now, lat, lon);
    const mp = SunCalc.getMoonIllumination(now);
    safeSet('sun-altaz', `${(sp.alt*180/Math.PI).toFixed(2)}° / ${(sp.az*180/Math.PI).toFixed(2)}°`);
    safeSet('moon-phase', `${(mp.phase*360).toFixed(1)}° • ${(mp.fraction*100).toFixed(1)}%`);
    // Equation of Time & solar mean date (approx)
    const year = now.getUTCFullYear(), month = now.getUTCMonth()+1, day = now.getUTCDate();
    safeSet('eot', ((-7.655*Math.sin(0.98565*(day))) ).toFixed(2)+' min'); // placeholder, see J2000 for exact
  }catch(e){ console.warn('sunmoon fail', e); }
}

/* -------------------- ISS polling -------------------- */
async function pollISS(){
  try{
    const j = await API.iss();
    if(j && j.iss_position){
      const lat = parseFloat(j.iss_position.latitude), lon = parseFloat(j.iss_position.longitude);
      safeSet('iss-last', new Date().toLocaleTimeString());
      issMarker?.setLatLng([lat, lon]);
      if(followIss) map?.panTo([lat, lon]);
      if(issTrailOn) issTrail?.addLatLng([lat, lon]);
      Storage.save({ts:nowISO(), source:'iss', lat, lon});
    }
  }catch(e){ console.warn('pollISS fail', e); }
  setTimeout(pollISS, 5000);
}

/* -------------------- Replay trail -------------------- */
async function replayTrail(){
  const rows = await Storage.all();
  if(!rows || rows.length===0) return alert('Aucun enregistrement');
  issTrail?.setLatLngs([]);
  let i=0;
  const timer = setInterval(()=>{
    if(i>=rows.length){ clearInterval(timer); return; }
    const r = rows[i++];
    if(r.lat && r.lon) issTrail?.addLatLng([r.lat, r.lon]);
  }, 150);
}

/* app.js — Partie 5/5
   Init sequence, final polish, plugin notes and proxy snippet
*/

/* -------------------- INIT -------------------- */
(async function init(){
  // initial UI defaults
  safeSet('local-time', new Date().toLocaleString());
  // default position Paris
  const lat0=48.8566, lon0=2.3522;
  try{ if(userMarker) userMarker.setLatLng([lat0, lon0]); if(map) map.setView([lat0, lon0], 5); } catch(e){}
  // initialize filter
  filterEngine.initFromGnss(lat0, lon0, 35);
  // start ISS polling
  pollISS();
  // initial refresh
  await orchestrator.runRefresh(lat0, lon0);
  // periodic derived update
  setInterval(()=> {
    const state = filterEngine.getState();
    if(state){ safeSet('lat', state.lat.toFixed(6)); safeSet('lon', state.lon.toFixed(6)); safeSet('alt', (state.alt||0).toFixed(2)+' m'); }
    refreshDerived(state?state.lat:lat0, state?state.lon:lon0);
  }, 7000);
})();

/* -------------------- POLISH: accessibility & fallbacks -------------------- */
/* Add ARIA where possible and minimal keyboard interactions */
document.querySelectorAll('button').forEach(b=>{ b.setAttribute('tabindex','0'); b.addEventListener('keyup', (e)=>{ if(e.key==='Enter') b.click(); }); });

/* -------------------- PLUGIN NOTES -------------------- */
/*
 - The code integrates a stable EKF by default.
 - If you want to use any advanced UKF/EKF variant found in the ZIP (for ex. "gnss-dashboard-full(...).js"),
   open that file, locate the filter class (UKF21, etc.), and provide it as plugin:
     const plugin = new YourUKFPlugin(params);
     filterEngine.instance = plugin;
     filterEngine.type = 'UKF21';
   Ensure plugin exposes: init(lat,lon,alt), predict(dt), updateGnss(lat,lon,alt,sigma), updateImu(acc,dt), getState()
 - All original experimental files from the ZIP are stored in extras/ (if you requested ZIP output).
*/

/* -------------------- PROXY SNIPPET (OPTIONAL) -------------------- */
/*
 server-proxy.js (Node/Express) - use to proxy http://api.open-notify.org to avoid mixed-content
 ------------------------------------------
 const express = require('express');
 const fetch = require('node-fetch');
 const app = express();
 const PORT = process.env.PORT || 8080;
 app.get('/iss-now.json', async (req,res) => {
   try {
     const r = await fetch('http://api.open-notify.org/iss-now.json');
     const body = await r.text();
     res.set('Content-Type','application/json'); res.send(body);
   } catch(e){ res.status(502).json({error:'proxy error'}); }
 });
 app.use((req,res,next)=>{ res.set('Access-Control-Allow-Origin','*'); next(); });
 app.listen(PORT, ()=> console.log('proxy on', PORT));
 ------------------------------------------
 Modify API.iss() to point to https://your-proxy/iss-now.json if you deploy it.
*/

/* -------------------- DONE -------------------- */
console.log('Dashboard app.js loaded — fusion complète');
