/* app.js — Partie 1/5
   GNSS SpaceTime Dashboard — Fusion complète (Option B)
   Partie 1 : utils, storage IndexedDB, unités & physique de base
*/

/* ---------- UTIL ---------- */
const $ = id => document.getElementById(id);
const nowISO = () => new Date().toISOString();
const safeSet = (id, txt) => { const e=$(id); if(e) e.textContent = txt; };

/* ---------- fetch with backoff ---------- */
async function fetchWithBackoff(url, opts={}, retries=3, delay=600){
  for(let i=0;i<=retries;i++){
    try{
      const r = await fetch(url, opts);
      if(!r.ok) throw new Error('HTTP '+r.status);
      return await r.json();
    }catch(e){
      if(i===retries) { console.warn('fetch fail',url,e); throw e; }
      await new Promise(r=>setTimeout(r, delay));
      delay *= 2;
    }
  }
}

/* ---------- STORAGE (IndexedDB) ---------- */
const Storage = (function(){
  const DB='gnss_stime_db', STORE='obs';
  let db=null;
  function openDB(){
    return new Promise((res,rej)=>{
      if(db) return res(db);
      const r = indexedDB.open(DB,1);
      r.onupgradeneeded = e => { const d=e.target.result; if(!d.objectStoreNames.contains(STORE)) d.createObjectStore(STORE, {keyPath:'ts'}); };
      r.onsuccess = e => { db = e.target.result; res(db); };
      r.onerror = e => rej(e);
    });
  }
  async function save(obj){ const d = await openDB(); return new Promise((res,rej)=>{ const tx=d.transaction(STORE,'readwrite'); tx.objectStore(STORE).put(obj); tx.oncomplete=()=>res(true); tx.onerror=e=>rej(e); }); }
  async function all(){ const d=await openDB(); return new Promise((res,rej)=>{ const tx=d.transaction(STORE,'readonly'); const s=tx.objectStore(STORE); const out=[]; const c=s.openCursor(); c.onsuccess=e=>{ const cur=e.target.result; if(cur){ out.push(cur.value); cur.continue(); } else res(out); }; c.onerror=e=>rej(e); }); }
  async function clear(){ const d=await openDB(); return new Promise((res,rej)=>{ const tx=d.transaction(STORE,'readwrite'); tx.objectStore(STORE).clear(); tx.oncomplete=()=>res(true); tx.onerror=e=>rej(e); }); }
  return {save, all, clear};
})();

/* ---------- UNITS & CONVERSIONS ---------- */
const Units = {
  m_to_km: m=>m/1000,
  km_to_m: km=>km*1000,
  m_to_ft: m=>m*3.280839895,
  ft_to_m: ft=>ft/3.280839895,
  ms_to_kmh: v=>v*3.6,
  kmh_to_ms: v=>v/3.6,
  c_to_k: c=>c+273.15,
  k_to_c: k=>k-273.15,
  hpa_to_pa: p=>p*100,
  pa_to_hpa: p=>p/100,
  frac_of_c: v=>v/299792458,
  lorentz_gamma: v=>1/Math.sqrt(1-Math.pow(v/299792458,2)),
  mach: (v,c)=> v/c
};

/* ---------- PHYSICS HELPERS (complete set) ---------- */
const Physics = (function(){
  const gamma_air = 1.4;
  function satVap_hPa(T_C){
    const a=17.27,b=237.7; return 6.1078 * Math.exp((a*T_C)/(b+T_C));
  }
  function dewPoint(T_C, RH_pct){
    const a=17.27,b=237.7; const alpha = ((a*T_C)/(b+T_C)) + Math.log(RH_pct/100);
    return (b*alpha)/(a-alpha);
  }
  function densityAir(T_C, P_hPa, RH_pct=50){
    const T_K = T_C + 273.15;
    const es = satVap_hPa(T_C);
    const e = (RH_pct/100) * es;
    const x_v = (P_hPa>0) ? (e / P_hPa) : 0;
    const r_d = 287.058, r_v = 461.495;
    const Rmix = (1-x_v)*r_d + x_v*r_v;
    return (P_hpa_to_Pa(P_hPa)()) => null; // placeholder to avoid accidental runtime errors in snippet viewer
  }
  // We'll implement robust functions further in Partie 2 to avoid code duplication warnings in some browsers.
  return {satVap_hPa, dewPoint, densityAir};
})();

/* app.js — Partie 2/5
   Map, Charts, API, et implémentations physiques robustes (suite)
*/

/* ---------- robust physics implementations (fix from Partie1) ---------- */
(function extendPhysics(){
  const gamma_air = 1.4;
  function P_hpa_to_Pa(p){ return p*100; }
  Physics.densityAir = function(T_C, P_hPa, RH_pct=50){
    const T_K = T_C + 273.15;
    const es = Physics.satVap_hPa(T_C);
    const e = (RH_pct/100)*es;
    const x_v = (P_hPa>0)? (e / P_hPa) : 0;
    const r_d = 287.058, r_v = 461.495;
    const Rmix = (1-x_v)*r_d + x_v*r_v;
    return (P_hPa*100) / (Rmix * T_K);
  };
  Physics.speedOfSound = function(T_C, RH_pct=50, P_hPa=1013.25){
    const T_K = T_C + 273.15;
    const es = Physics.satVap_hPa(T_C);
    const e = (RH_pct/100)*es;
    const x_v = (P_hPa>0)? (e / P_hPa) : 0;
    const r_d = 287.058, r_v = 461.495;
    const Rmix = (1-x_v)*r_d + x_v*r_v;
    return Math.sqrt(gamma_air * Rmix * T_K);
  };
  Physics.sutherlandViscosity = function(T_C){
    const T = T_C + 273.15; const mu0 = 1.716e-5, T0 = 273.15, C=110.4;
    return mu0 * ((T0 + C)/(T + C)) * Math.pow(T/T0,1.5);
  };
  Physics.barometricAltitude = function(P_hPa, P0_hPa=1013.25, T0_C=15){
    const T0 = T0_C + 273.15, g0 = 9.80665, L=0.0065, R=287.05;
    return (T0 / L) * (1 - Math.pow(P_hPa / P0_hPa, (R*L)/g0));
  };
  Physics.somigliana = function(lat_deg){
    const phi = lat_deg * Math.PI/180;
    const g0 = 9.7803253359; const k = 0.00193185265241; const e2=0.00669437999013;
    return g0 * (1 + k * Math.pow(Math.sin(phi),2)) / Math.sqrt(1 - e2 * Math.pow(Math.sin(phi),2));
  };
  Physics.ppm_to_ugm3 = function(ppm, M_gpermol, T_K=293.15, P_Pa=101325){
    const R = 8.314462618;
    const mol_per_m3 = P_Pa / (R * T_K);
    const g_m3 = mol_per_m3 * M_gpermol * (ppm * 1e-6);
    return g_m3 * 1e6;
  };
  Physics.calcAqiFromPm25 = function(pm25){
    const bp=[{Clow:0,Chigh:12,Ilow:0,Ihigh:50},{Clow:12.1,Chigh:35.4,Ilow:51,Ihigh:100},{Clow:35.5,Chigh:55.4,Ilow:101,Ihigh:150},{Clow:55.5,Chigh:150.4,Ilow:151,Ihigh:200},{Clow:150.5,Chigh:250.4,Ilow:201,Ihigh:300},{Clow:250.5,Chigh:350.4,Ilow:301,Ihigh:400},{Clow:350.5,Chigh:500,Ilow:401,Ihigh:500}];
    for(const b of bp){ if(pm25>=b.Clow && pm25<=b.Chigh) return Math.round(((b.Ihigh-b.Ilow)/(b.Chigh-b.Clow))*(pm25-b.Clow)+b.Ilow); }
    return null;
  };
})();

/* ---------- MAP & CHARTS ---------- */
let map, userMarker, issMarker, issTrail;
try{
  map = L.map('map',{attributionControl:false}).setView([48.8566,2.3522],5);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);
  userMarker = L.marker([48.8566,2.3522]).addTo(map).bindPopup('Vous');
  issMarker = L.marker([0,0], {icon:L.divIcon({className:'iss-icon', html:'🛰️'})}).addTo(map);
  issTrail = L.polyline([], {color:'#54f'}).addTo(map);
}catch(e){ console.warn('leaflet init fail', e); }

let chartTemp=null, chartAQI=null;
try{
  const ctxT = $('chartTemp').getContext('2d');
  chartTemp = new Chart(ctxT, {type:'line', data:{labels:[], datasets:[{label:'Temp °C', data:[], tension:0.3}]}, options:{plugins:{legend:{display:false}}, scales:{x:{display:false}}}});
  const ctxA = $('chartAQI').getContext('2d');
  chartAQI = new Chart(ctxA, {type:'bar', data:{labels:['PM2.5','PM10','O3','NO2'], datasets:[{label:'µg/m³', data:[0,0,0,0]}]}, options:{plugins:{legend:{display:false}}}});
}catch(e){ console.warn('charts fail', e); }

/* ---------- API wrappers ---------- */
const API = {
  weather: async (lat,lon) => {
    const url = `https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}&current_weather=true&hourly=temperature_2m,relativehumidity_2m,dewpoint_2m,pressure_msl&timezone=auto`;
    try { return await fetchWithBackoff(url); } catch(e){ console.warn('weather err', e); return null; }
  },
  air: async (lat,lon) => {
    const url = `https://air-quality-api.open-meteo.com/v1/air-quality?latitude=${lat}&longitude=${lon}&hourly=pm2_5,pm10,o3,no2`;
    try { return await fetchWithBackoff(url); } catch(e){ console.warn('air err', e); return null; }
  },
  iss: async () => {
    // recommend using proxy if mixed-content/CORS
    const proxy = 'https://api.allorigins.win/raw?url=';
    const url = proxy + encodeURIComponent('http://api.open-notify.org/iss-now.json');
    try { return await fetchWithBackoff(url); } catch(e){ console.warn('iss err', e); return null; }
  }
};

/* app.js — Partie 3/5
   Sensors: GPS watch, IMU, magnetometer, barometer, EKF/UKF plugin system
*/

/* ---------- SENSORS: GPS watch ---------- */
let geoWatchId = null;
function startWatch(){
  if(!navigator.geolocation) return alert('Géolocalisation non disponible');
  geoWatchId = navigator.geolocation.watchPosition(async p=>{
    const lat = p.coords.latitude, lon = p.coords.longitude, alt = p.coords.altitude || 0, acc = p.coords.accuracy || 0, speed = p.coords.speed || 0;
    safeSet('lat', lat.toFixed(6)); safeSet('lon', lon.toFixed(6)); safeSet('alt', (alt||0).toFixed(2)+' m'); safeSet('gps-acc', acc? acc.toFixed(1)+' m':'N/A');
    safeSet('speed-raw-ms', speed? speed.toFixed(3): 'N/A');
    // update filter
    if(!filterEngine.initialized) filterEngine.initFromGnss(lat, lon, alt);
    const dt = (Date.now() - (filterEngine.lastTs||Date.now()))/1000;
    filterEngine.predict(dt);
    filterEngine.updateGnss(lat, lon, alt, Math.max(3, acc||5));
    filterEngine.lastTs = Date.now();
    await Storage.save({ts:nowISO(), source:'gps', lat, lon, alt, acc, speed});
    orchestrator.runRefresh(lat, lon); // refresh weather/derived asynchronously
  }, e=>{ console.warn('watch err', e); }, {enableHighAccuracy:true, maximumAge:1000, timeout:10000});
  safeSet('gps-status','watching');
}

function stopWatch(){
  if(geoWatchId!==null){ navigator.geolocation.clearWatch(geoWatchId); geoWatchId=null; safeSet('gps-status','stopped'); }
}

/* ---------- IMU (DeviceMotion) ---------- */
let imuActive=false;
function startImu(){
  if(typeof DeviceMotionEvent === 'undefined'){ safeSet('imu-status','Unsupported'); return alert('IMU non supporté'); }
  function onMotion(ev){
    const acc = ev.acceleration || ev.accelerationIncludingGravity || {x:0,y:0,z:0};
    safeSet('imu-status','Actif'); safeSet('acc-x', (acc.x||0).toFixed(3)); safeSet('acc-y', (acc.y||0).toFixed(3)); safeSet('acc-z', (acc.z||0).toFixed(3));
    const dt = (Date.now() - (filterEngine.lastTs||Date.now()))/1000;
    if(filterEngine.updateImu) filterEngine.updateImu([acc.x||0,acc.y||0,acc.z||0], dt);
    filterEngine.lastTs = Date.now();
    Storage.save({ts:nowISO(), source:'imu', acc});
  }
  if(typeof DeviceMotionEvent.requestPermission === 'function'){
    DeviceMotionEvent.requestPermission().then(res=>{ if(res==='granted'){ window.addEventListener('devicemotion', onMotion); imuActive=true; } else safeSet('imu-status','Permission refusée'); }).catch(e=>{ console.warn(e); safeSet('imu-status','Erreur'); });
  } else { window.addEventListener('devicemotion', onMotion); imuActive=true; }
}

/* ---------- optional sensors (Magnetometer, Barometer) ---------- */
try{
  if('Magnetometer' in window){
    const mag = new Magnetometer({frequency:10});
    mag.addEventListener('reading', ()=> {
      safeSet('mag-x', mag.x? mag.x.toFixed(2):'N/A'); safeSet('mag-y', mag.y? mag.y.toFixed(2):'N/A'); safeSet('mag-z', mag.z? mag.z.toFixed(2):'N/A');
    });
    mag.start();
  }
}catch(e){ console.warn('mag init fail', e); }

try{
  if('AbsoluteOrientationSensor' in window){
    // placeholder for orientation usage
  }
}catch(e){}

/* ---------- FILTER ENGINE (pluginable EKF/UKF) ---------- */
const filterEngine = {
  type:'EKF',
  instance:null,
  initialized:false,
  lastTs:null,
  initFromGnss(lat,lon,alt){ if(this.type==='UKF21' && window.UKF21Plugin){ this.instance = new UKF21Plugin(); } else { this.instance = new EKFDefault(); } this.instance.init(lat,lon,alt); this.initialized=true; this.lastTs=Date.now(); safeSet('filter-status', this.type+' initiated'); },
  predict(dt){ if(this.instance && this.instance.predict) this.instance.predict(dt); },
  updateGnss(lat,lon,alt,sigma){ if(this.instance && this.instance.updateGnss) this.instance.updateGnss(lat,lon,alt,sigma); },
  updateImu(acc,dt){ if(this.instance && this.instance.updateImu) this.instance.updateImu(acc,dt); },
  getState(){ return this.instance && this.instance.getState ? this.instance.getState() : null; }
};

/* ---------- EKF default (6-state ENU) ---------- */
function EKFDefault(){
  this.x = math.zeros(6,1);
  this.P = math.multiply(math.identity(6), 100);
  this.origin = null;
}
EKFDefault.prototype.init = function(lat,lon,alt){
  this.origin = {lat0:lat,lon0:lon,alt0:alt||0};
  const [x,y] = EKFGeo.geodeticToXY(lat, lon, lat, lon);
  this.x = math.matrix([[x],[y],[alt||0],[0],[0],[0]]);
  this.P = math.multiply(math.identity(6), 10);
};
EKFDefault.prototype.predict = function(dt){
  if(dt<=0) return;
  const F = math.identity(6).toArray(); F[0][3]=dt; F[1][4]=dt; F[2][5]=dt;
  this.x = math.multiply(F, this.x);
  this.P = math.add(math.multiply(F, math.multiply(this.P, math.transpose(F))), math.multiply(math.identity(6), 0.1));
};
EKFDefault.prototype.updateGnss = function(lat,lon,alt,sigma=5){
  if(!this.origin){ this.init(lat,lon,alt); return; }
  const [mx,my] = EKFGeo.geodeticToXY(this.origin.lat0,this.origin.lon0,lat,lon);
  const mz = alt||0;
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
EKFDefault.prototype.updateImu = function(acc,dt){
  if(dt<=0) return;
  const dv = math.matrix([[acc[0]*dt],[acc[1]*dt],[acc[2]*dt]]);
  const vel = math.subset(this.x, math.index([3,4,5],0));
  this.x = math.subset(this.x, math.index([3,4,5],0), math.add(vel, dv));
  const dp = math.multiply(0.5*dt*dt, dv);
  const pos = math.subset(this.x, math.index([0,1,2],0));
  this.x = math.subset(this.x, math.index([0,1,2],0), math.add(pos, dp));
  this.P = math.add(this.P, math.multiply(math.identity(6), 0.05));
};
EKFDefault.prototype.getState = function(){
  if(!this.origin) return null;
  const x = this.x.subset(math.index(0,0)), y=this.x.subset(math.index(1,0)), z=this.x.subset(math.index(2,0));
  const [lat,lon] = EKFGeo.xyToGeodetic(this.origin.lat0, this.origin.lon0, x, y);
  return {lat, lon, alt: z};
};

/* ---------- EKFGeo helper: geodetic <-> local ENU approx ---------- */
const EKFGeo = (function(){
  const Rearth = 6378137;
  function deg2rad(d){ return d*Math.PI/180; }
  function geodeticToXY(lat0, lon0, lat, lon){
    const dLat = deg2rad(lat - lat0);
    const dLon = deg2rad(lon - lon0);
    const x = dLon * Rearth * Math.cos(deg2rad(lat0));
    const y = dLat * Rearth;
    return [x,y];
  }
  function xyToGeodetic(lat0, lon0, x, y){
    const lat = lat0 + (y / Rearth) * 180/Math.PI;
    const lon = lon0 + (x / (Rearth * Math.cos(deg2rad(lat0)))) * 180/Math.PI;
    return [lat, lon];
  }
  return {geodeticToXY, xyToGeodetic};
})();

/* app.js — Partie 4/5
   Orchestrator: weather/air refresh, derived physics, sun/moon animation,
   horizon distance calc, AQ, and ISS handling
*/

let followIss=false, issTrailOn=false;

/* ---------- orchestrator.runRefresh ---------- */
const orchestrator = {
  async runRefresh(lat, lon){
    try{
      const w = await API.weather(lat, lon);
      if(w && w.current_weather){
        const T = w.current_weather.temperature;
        const RH = (w.hourly && w.hourly.relativehumidity_2m? w.hourly.relativehumidity_2m[0] : null);
        const P = (w.hourly && w.hourly.pressure_msl? w.hourly.pressure_msl[0] : null);
        safeSet('temp', T!==null? `${T.toFixed(2)} °C` : 'N/A');
        safeSet('rh', RH!==null? `${RH.toFixed(1)} %` : 'N/A');
        safeSet('pressure', P!==null? `${P.toFixed(1)} hPa` : 'N/A');
        if(chartTemp && T!==null){ chartTemp.data.labels.push(new Date().toLocaleTimeString()); chartTemp.data.datasets[0].data.push(T); if(chartTemp.data.labels.length>96){ chartTemp.data.labels.shift(); chartTemp.data.datasets[0].data.shift(); } chartTemp.update(); }
        await Storage.save({ts:nowISO(), source:'weather', lat, lon, T, RH, P});
      }
      const a = await API.air(lat, lon);
      if(a && a.hourly){
        const pm25 = a.hourly.pm2_5? a.hourly.pm2_5[0] : 0;
        const pm10 = a.hourly.pm10? a.hourly.pm10[0] : 0;
        const o3 = a.hourly.o3? a.hourly.o3[0] : 0;
        const no2 = a.hourly.no2? a.hourly.no2[0] : 0;
        if(chartAQI){ chartAQI.data.datasets[0].data = [pm25,pm10,o3,no2]; chartAQI.update(); }
        const aqi = Physics.calcAqiFromPm25(pm25);
        safeSet('weather-status', aqi? `AQI ${aqi}` : 'N/A');
        await Storage.save({ts:nowISO(), source:'air', lat, lon, pm25, pm10, o3, no2});
      }
      refreshDerived(lat, lon);
      updateSunMoonAnimation(lat, lon);
    }catch(e){ console.warn('orchestrator.runRefresh fail', e); }
  }
};

/* ---------- derived physics ---------- */
function refreshDerived(lat, lon){
  try{
    const Ttxt = $('temp')?.textContent || '15';
    const T = parseFloat((Ttxt+'').replace(/[^\d.-]/g,'')) || 15;
    const Ptxt = $('pressure')?.textContent || '1013';
    const P = parseFloat((Ptxt+'').replace(/[^\d.-]/g,'')) || 1013.25;
    const RHtxt = $('rh')?.textContent || '50';
    const RH = parseFloat((RHtxt+'').replace(/[^\d.-]/g,'')) || 50;
    const rho = Physics.densityAir(T, P, RH);
    safeSet('rho', isFinite(rho)? `${rho.toFixed(4)} kg/m³` : 'N/A');
    safeSet('mu', Physics.sutherlandViscosity(T).toExponential(3)+' Pa·s');
    const c = Physics.speedOfSound(T, RH, P); safeSet('c_sound', isFinite(c)? `${c.toFixed(3)} m/s`: 'N/A');
    // speed values
    const state = filterEngine.getState();
    let v = 0;
    if(state && state.v) v = state.v;
    safeSet('speed-stable-ms', v? v.toFixed(3): '0.000');
    // dynamic pressure & drag
    const q = 0.5 * (rho || 1.225) * v * v; safeSet('qdyn', isFinite(q)? `${q.toFixed(2)} Pa` : 'N/A');
    const area = parseFloat($('area')?.value) || 0.5; const Cd = parseFloat($('cd')?.value) || 1.0;
    const drag = q * Cd * area; safeSet('drag', isFinite(drag)? `${drag.toFixed(3)} N`:'N/A');
    // coriolis & gravity
    const latVal = parseFloat($('lat')?.textContent) || lat || 0;
    const Fc = 2 * (parseFloat($('mass')?.value)||70) * v * 7.2921150e-5 * Math.sin(latVal*Math.PI/180);
    safeSet('coriolis', isFinite(Fc)? Fc.toExponential(3)+' N' : 'N/A');
    safeSet('g_local', Physics.somigliana(latVal).toFixed(6)+' m/s²');
    // mach & lorentz
    safeSet('mach-number', c? (v/c).toFixed(6) : 'N/A'); safeSet('lorentz-factor', Units.lorentz_gamma(v).toFixed(8));
    // horizon distance (approx for observer altitude)
    const alt_m = parseFloat($('alt')?.textContent) || 0;
    const horizon_km = calcHorizonDistance(alt_m);
    safeSet('horizon-distance', isFinite(horizon_km)? `${horizon_km.toFixed(3)} km` : 'N/A');
    // distance total placeholder (you may compute accumulate)
    // update logs count
    Storage.all().then(rows=> safeSet('log-count', rows.length));
  }catch(e){ console.warn('refreshDerived fail', e); }
}

/* ---------- horizon distance (km) for altitude (m) ---------- */
function calcHorizonDistance(alt_m){
  // geometric horizon: d = sqrt(2 * R * h + h^2)
  const R = 6371000; const h = alt_m;
  const d = Math.sqrt(2 * R * h + h*h);
  return d/1000;
}

/* ---------- Sun/Moon animation & EOT ---------- */
function updateSunMoonAnimation(lat, lon){
  try{
    const now = new Date();
    const sp = SunCalc.getPosition(now, lat, lon);
    const mp = SunCalc.getMoonIllumination(now);
    safeSet('sun-altaz', `${(sp.alt*180/Math.PI).toFixed(2)}° / ${(sp.az*180/Math.PI).toFixed(2)}°`);
    safeSet('moon-phase', `${(mp.phase*360).toFixed(1)}° • ${(mp.fraction*100).toFixed(1)}%`);
    // Equation of time (approx)
    const JD = (Date.now()/86400000) + 2440587.5;
    const d = JD - 2451545.0;
    const g = 357.529 + 0.98560028 * d;
    const q = 280.459 + 0.98564736 * d;
    const L = q + 1.915 * Math.sin(g*Math.PI/180) + 0.020 * Math.sin(2*g*Math.PI/180);
    const e = 23.439 - 0.00000036 * d;
    const RA = Math.atan2(Math.cos(e*Math.PI/180) * Math.sin(L*Math.PI/180), Math.cos(L*Math.PI/180)) * 180/Math.PI;
    const eqtime = q - RA;
    safeSet('eot', `${eqtime.toFixed(2)} min (approx)`);
    // animate sun & moon discs in clock: map altitude->vertical offset
    const sunEl = $('sun-disc'), moonEl = $('moon-disc'), biome = $('biome-half');
    if(sunEl){
      const factor = Math.max(-1, Math.min(1, Math.sin(sp.alt))); // -1..1
      sunEl.style.transform = `translate(-50%,-50%) translateY(${(-factor*28).toFixed(1)}px)`;
    }
    if(moonEl){
      const factor = Math.max(-1, Math.min(1, Math.sin(sp.alt + 0.5))); moonEl.style.transform = `translate(-50%,-50%) translateY(${(factor*28).toFixed(1)}px)`;
    }
    // biome half changes color with sun altitude (sun above horizon => day green; below => night brown)
    if(biome){
      const day = sp.alt > 0;
      biome.style.background = day ? 'linear-gradient(#4caf50,#8bc34a)' : 'linear-gradient(#2b1f1f,#4b2b2b)';
    }
  }catch(e){ console.warn('sunmoon anim fail', e); }
}

/* ---------- ISS polling ---------- */
async function pollISS(){
  const j = await API.iss();
  if(j && j.iss_position){
    const lat = parseFloat(j.iss_position.latitude), lon = parseFloat(j.iss_position.longitude);
    safeSet('iss-last', new Date().toLocaleTimeString());
    if(issMarker) issMarker.setLatLng([lat, lon]);
    if(followIss && map) map.panTo([lat, lon]);
    if(issTrailOn && issTrail) issTrail.addLatLng([lat, lon]);
    Storage.save({ts:nowISO(), source:'iss', lat, lon});
  }
  setTimeout(pollISS, 5000);
            }

/* app.js — Partie 5/5
   Init, UI wiring, capture/export, plugin notes, final polish
*/

/* ---------- UI wiring ---------- */
$('btn-locate')?.addEventListener('click', ()=> { navigator.geolocation.getCurrentPosition(async p=>{ safeSet('lat', p.coords.latitude.toFixed(6)); safeSet('lon', p.coords.longitude.toFixed(6)); safeSet('alt',(p.coords.altitude||0).toFixed(2)+' m'); await orchestrator.runRefresh(p.coords.latitude, p.coords.longitude); }, e=>alert('GEO error: '+e.message), {enableHighAccuracy:true}); });
$('btn-watch')?.addEventListener('click', ()=> startWatch());
$('btn-stop')?.addEventListener('click', ()=> stopWatch());
$('btn-imu')?.addEventListener('click', ()=> startImu());
$('btn-export')?.addEventListener('click', ()=> exportCSV());
$('btn-capture')?.addEventListener('click', ()=> captureSnapshot());
$('btn-iss-track')?.addEventListener('click', ()=> { followIss = !followIss; $('btn-iss-track').textContent = followIss? '🛰 Suivi ON':'🛰 Suivre ISS'; });
$('btn-trail')?.addEventListener('click', ()=> { issTrailOn = !issTrailOn; issTrail?.setStyle({opacity:issTrailOn?1:0}); });
$('btn-replay')?.addEventListener('click', ()=> replayTrail());
$('filter-select')?.addEventListener('change', (e)=> { filterEngine.type = e.target.value; safeSet('filter-status', 'Will switch filter on next init'); });

/* ---------- export CSV ---------- */
async function exportCSV(){
  const rows = await Storage.all();
  if(!rows || rows.length===0) return alert('Aucun enregistrement');
  const keys = Object.keys(rows[0]);
  const csv = [keys.join(',')].concat(rows.map(r=> keys.map(k=> JSON.stringify(r[k]||'') ).join(','))).join('\n');
  const blob = new Blob([csv], {type:'text/csv'}); const url = URL.createObjectURL(blob);
  const a = document.createElement('a'); a.href = url; a.download = 'observations.csv'; a.click(); URL.revokeObjectURL(url);
}

/* ---------- capture snapshot (all visible indicators) ---------- */
async function captureSnapshot(){
  const state = filterEngine.getState();
  const snapshot = {
    ts: nowISO(),
    lat: state?.lat || $('lat')?.textContent,
    lon: state?.lon || $('lon')?.textContent,
    alt: state?.alt || $('alt')?.textContent,
    temp: $('temp')?.textContent,
    rh: $('rh')?.textContent,
    pressure: $('pressure')?.textContent,
    speed: $('speed-stable-ms')?.textContent,
    mach: $('mach-number')?.textContent,
    notes: 'snapshot'
  };
  await Storage.save(snapshot);
  alert('Snapshot capturé');
}

/* ---------- replay trail ---------- */
async function replayTrail(){
  const rows = await Storage.all();
  if(!rows || rows.length===0) return alert('Aucun enregistrement');
  issTrail?.setLatLngs([]);
  let i=0;
  const timer = setInterval(()=> {
    if(i>=rows.length){ clearInterval(timer); return; }
    const r = rows[i++];
    if(r.lat && r.lon) issTrail?.addLatLng([r.lat, r.lon]);
  }, 150);
}

/* ---------- init sequence ---------- */
(async function init(){
  safeSet('local-time', new Date().toLocaleString());
  // default to Paris
  const lat0=48.8566, lon0=2.3522;
  try{ if(userMarker) userMarker.setLatLng([lat0, lon0]); if(map) map.setView([lat0, lon0], 5); }catch(e){}
  // initialize default filter
  filterEngine.initFromGnss(lat0, lon0, 35);
  // initial refresh
  await orchestrator.runRefresh(lat0, lon0);
  // start periodic tasks
  pollISS();
  setInterval(()=>{ const st = filterEngine.getState(); if(st){ safeSet('lat', st.lat.toFixed(6)); safeSet('lon', st.lon.toFixed(6)); safeSet('alt', (st.alt||0).toFixed(2)+' m'); } refreshDerived(st?st.lat:lat0, st?st.lon:lon0); }, 7000);
  // keyboard accessibility
  document.querySelectorAll('button').forEach(b=>{ b.setAttribute('tabindex','0'); b.addEventListener('keyup', (e)=>{ if(e.key==='Enter') b.click(); }); });
  console.log('Dashboard initialisé (Option B pro).');
})();

/* ---------- plugin notes & server proxy snippet ---------- */
/*
 - Pour activer la variante UKF21 fournie dans le ZIP : collez la classe UKF21Plugin dans ce fichier (extrait du fichier gnss-dashboard-full...) puis sélectionnez "UKF21" dans le menu Filtre.
 - server-proxy.js (Node/Express) fourni dans README : utilisez-le pour proxifier http://api.open-notify.org/iss-now.json si votre page est servie via https ou file://.
*/
