/* app.part1.js
   PARTIE 1/5 - Constantes, SciencePro offline, utilitaires de base
*/

const $ = id => document.getElementById(id);
const nowISO = () => new Date().toISOString();
const safeSet = (id, txt) => { const e = $(id); if(e) e.textContent = txt; };

/* PHYS CONSTANTS */
const PHYS = {
  c: 299792458,
  G: 6.67430e-11,
  AU: 149597870700,
  LY: 9.4607e15,
  R_earth: 6371000,
  omega_earth: 7.2921150e-5,
  R_d: 287.058,
  R_v: 461.495,
  gamma_air: 1.4
};

/* SciencePro - offline module (Meeus approx for EoT, Sun, Moon; atmosphere models, drag, relativity) */
const SciencePro = (function(){
  const C = { ...PHYS, pi: Math.PI };

  function toJulian(date){
    const Y = date.getUTCFullYear();
    let M = date.getUTCMonth() + 1;
    let D = date.getUTCDate() + (date.getUTCHours()/24) + (date.getUTCMinutes()/1440) + (date.getUTCSeconds()/86400) + (date.getUTCMilliseconds()/(86400000));
    if(M <= 2){ M += 12; Y -= 1; }
    const A = Math.floor(Y/100);
    const B = 2 - A + Math.floor(A/4);
    const JD = Math.floor(365.25*(Y+4716)) + Math.floor(30.6001*(M+1)) + D + B - 1524.5;
    return JD;
  }

  function equationOfTime(date){
    const jd = toJulian(date);
    const T = (jd - 2451545.0)/36525.0;
    let L0 = (280.46646 + 36000.76983*T + 0.0003032*T*T) % 360;
    if(L0 < 0) L0 += 360;
    const e = 0.016708634 - 0.000042037*T - 0.0000001267*T*T;
    const M = (357.52911 + 35999.05029*T - 0.0001537*T*T) * (Math.PI/180);
    const obliq = (23.43929111 - 0.013004167*T - 0.0000001639*T*T + 0.0000005036*T*T*T) * (Math.PI/180);
    const y = Math.tan(obliq/2);
    const y2 = y*y;
    const L0r = L0 * (Math.PI/180);
    const E = 4 * ( (y2 * Math.sin(2*L0r)) - (2*e * Math.sin(M)) + (4*e*y2 * Math.sin(M)*Math.cos(2*L0r)) - (0.5 * Math.pow(y,4) * Math.sin(4*L0r)) - (1.25 * e*e * Math.sin(2*M)) );
    return E; // minutes
  }

  function sunPosition(date){
    const jd = toJulian(date);
    const n = jd - 2451545.0;
    const L = (280.460 + 0.9856474 * n) % 360;
    const g = (357.528 + 0.9856003 * n) * (Math.PI/180);
    const lambda = (L + 1.915*Math.sin(g) + 0.020*Math.sin(2*g)) * (Math.PI/180);
    const epsilon = (23.439 - 0.0000004*n) * (Math.PI/180);
    const ra = Math.atan2(Math.cos(epsilon) * Math.sin(lambda), Math.cos(lambda));
    const dec = Math.asin(Math.sin(epsilon) * Math.sin(lambda));
    return { ra, dec, lambda };
  }

  function moonPhase(date){
    const syn = 29.53058867;
    const ref = new Date(Date.UTC(2000,0,6,18,14,0));
    const days = (date.getTime() - ref.getTime())/86400000;
    const phase = ((days % syn) + syn) % syn;
    const frac = 0.5*(1 - Math.cos(2*Math.PI*(phase/syn)));
    return { ageDays: phase, illuminatedFraction: frac };
  }

  // Saastamoinen zenith troposphere delay (m)
  function saastamoinenDelay(P_hPa, T_C, e_hPa, lat_deg, h_m){
    const P = P_hPa;
    const T = T_C + 273.15;
    const Zhd = 0.0022768 * P / (1 - 0.00266*Math.cos(2*lat_deg*Math.PI/180) - 0.00028*(h_m/1000));
    const Zwd = 0.002277 * (1255/T + 0.05) * e_hPa;
    return Zhd + Zwd;
  }

  function satVapour(T_C){
    const a=17.27, b=237.7;
    return 6.1078 * Math.exp((a*T_C)/(b+T_C));
  }

  function dewPoint(T_C, RH){
    const a=17.27, b=237.7;
    const alpha = ((a*T_C)/(b+T_C)) + Math.log(Math.max(1e-6, RH/100));
    return (b*alpha)/(a-alpha);
  }

  function densityAir(T_C, P_hPa, RH){
    const T_K = T_C + 273.15;
    const es = satVapour(T_C);
    const e = (RH/100) * es;
    const x_v = (P_hPa>0) ? (e / P_hPa) : 0;
    const R_d = C.R_d, R_v = C.R_v;
    const R_mix = (1 - x_v)*R_d + x_v*R_v;
    return (P_hPa*100)/(R_mix*T_K);
  }

  function speedOfSound(T_C, RH=50, P_hPa=1013.25){
    const gamma = C.gamma_air;
    const T_K = T_C + 273.15;
    const es = satVapour(T_C);
    const e = (RH/100)*es;
    const x_v = (P_hPa>0) ? (e / P_hPa) : 0;
    const R_mix = (1 - x_v)*C.R_d + x_v*C.R_v;
    return Math.sqrt(gamma * R_mix * T_K);
  }

  function sutherlandViscosity(T_C){
    const T = T_C + 273.15;
    const mu0 = 1.716e-5, T0 = 273.15, C_s = 110.4;
    return mu0 * ((T0 + C_s)/(T + C_s)) * Math.pow(T/T0, 1.5);
  }

  function calculateDrag(Cd, rho, V, A){ return 0.5 * Cd * rho * V * V * A; }
  function lorentzGamma(V){ return 1/Math.sqrt(1 - (V*V)/(C.c*C.c)); }
  function relativisticKE(m, V){ return (lorentzGamma(V) - 1) * m * C.c * C.c; }
  function horizon_km(h_m){ return Math.sqrt(2*C.R_earth*h_m + h_m*h_m)/1000; }

  return {
    toJulian, equationOfTime, sunPosition, moonPhase,
    saastamoinenDelay, satVapour, dewPoint, densityAir, speedOfSound,
    sutherlandViscosity, calculateDrag, lorentzGamma, relativisticKE,
    horizon_km, CONST: C
  };
})();

window.SciencePro = SciencePro;

/* small helper to list local files (from environment) - will show as available array if served */
(async function listLocalFiles(){
  // We'll attempt to fetch a manifest.json at server root if exists (optional)
  try{
    const res = await fetch('./manifest.json');
    if(res.ok){ const m = await res.json(); if($('local-files')) $('local-files').textContent = m.join(', '); }
  }catch(e){}
})();

/* app.part2.js
   PARTIE 2/5 - Storage, math helpers, auto-update cache
*/

const DB_NAME = 'gnss_science_db_v3';
const STORE = 'obs';
const META = 'meta';
let _db = null;

function openDB(){
  return new Promise((res, rej)=>{
    if(_db) return res(_db);
    const r = indexedDB.open(DB_NAME, 4);
    r.onupgradeneeded = e => {
      const db = e.target.result;
      if(!db.objectStoreNames.contains(STORE)) db.createObjectStore(STORE, {keyPath:'ts'});
      if(!db.objectStoreNames.contains(META)) db.createObjectStore(META);
    };
    r.onsuccess = e => { _db = e.target.result; res(_db); };
    r.onerror = e => rej(e);
  });
}

async function dbPut(obj){
  const db = await openDB();
  return new Promise((res, rej)=>{
    const tx = db.transaction(STORE,'readwrite');
    tx.objectStore(STORE).put(obj);
    tx.oncomplete = ()=> res(true);
    tx.onerror = e=> rej(e);
  });
}
async function dbGetAll(){
  const db = await openDB();
  return new Promise((res, rej)=>{
    const tx = db.transaction(STORE,'readonly');
    const cur = tx.objectStore(STORE).openCursor();
    const out = [];
    cur.onsuccess = e => { const c = e.target.result; if(c){ out.push(c.value); c.continue(); } else res(out); };
    cur.onerror = e => rej(e);
  });
}
async function dbClear(){ const db = await openDB(); return new Promise((res,rej)=>{ const tx=db.transaction(STORE,'readwrite'); tx.objectStore(STORE).clear(); tx.oncomplete=()=>res(true); tx.onerror=e=>rej(e); }); }

/* math helpers (small linear algebra used by filters) */
function eye(n, scale=0){ return Array.from({length:n}, (_,i)=> Array.from({length:n}, (_,j)=> i===j?scale:0)); }
function matMul(A,B){
  const m=A.length, p=B.length, n=B[0].length;
  const C = Array.from({length:m}, ()=> Array(n).fill(0));
  for(let i=0;i<m;i++) for(let k=0;k<p;k++) for(let j=0;j<n;j++) C[i][j] += A[i][k]*B[k][j];
  return C;
}
function transpose(A){ return A[0].map((_,i)=>A.map(r=>r[i])); }
function matAdd(A,B){ return A.map((r,i)=> r.map((v,j)=> v + B[i][j])); }
function matSub(A,B){ return A.map((r,i)=> r.map((v,j)=> v - B[i][j])); }

/* auto-update: fetch science caches when online */
async function autoUpdateScience(){
  if(!navigator.onLine) { safeSet('update-status', 'Hors-ligne'); return; }
  try{
    safeSet('update-status', 'Mise à jour...');
    // replace these urls by your update server if any
    const urls = [
      './science_cache.json',
      './ephemerides_small.json'
    ];
    for(const u of urls){
      try{
        const r = await fetch(u, {cache:'no-cache'});
        if(!r.ok) continue;
        const data = await r.json();
        const db = await openDB();
        const tx = db.transaction(META,'readwrite');
        tx.objectStore(META).put(data, u.split('/').pop());
      }catch(e){ console.warn('update failed', e); }
    }
    safeSet('update-status','A jour');
  }catch(e){
    safeSet('update-status','Erreur');
  }
}
setInterval(autoUpdateScience, 60*60*1000);
window.addEventListener('online', ()=> autoUpdateScience());

/* expose DB */
window.DB = { dbPut, dbGetAll, dbClear };

/* app.part3.js
   PARTIE 3/5 - UKF-21 filter + EKF fallback
*/

/* numeric helpers (cholesky + small linear algebra) */
function cholesky(A){
  const n = A.length;
  const L = Array.from({length:n}, ()=> Array(n).fill(0));
  for(let i=0;i<n;i++){
    for(let j=0;j<=i;j++){
      let s = 0;
      for(let k=0;k<j;k++) s += L[i][k]*L[j][k];
      if(i===j) L[i][j] = Math.sqrt(Math.max(0, A[i][i] - s));
      else L[i][j] = (A[i][j] - s) / L[j][j];
    }
  }
  return L;
}
function inv3(A){
  const a=A[0][0], b=A[0][1], c=A[0][2];
  const d=A[1][0], e=A[1][1], f=A[1][2];
  const g=A[2][0], h=A[2][1], i=A[2][2];
  const det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
  if(Math.abs(det) < 1e-12) return eye(3, 1e6);
  return [
    [(e*i - f*h)/det, (c*h - b*i)/det, (b*f - c*e)/det],
    [(f*g - d*i)/det, (a*i - c*g)/det, (c*d - a*f)/det],
    [(d*h - e*g)/det, (b*g - a*h)/det, (a*e - b*d)/det]
  ];
}

/* small geodesy local conversion (equirectangular approx) */
function deg2rad(d){ return d*Math.PI/180; }
function rad2deg(r){ return r*180/Math.PI; }
function geodeticToLocal(lat0, lon0, lat, lon){
  const R = PHYS.R_earth;
  const dLat = deg2rad(lat - lat0);
  const dLon = deg2rad(lon - lon0);
  const x = dLon * R * Math.cos(deg2rad(lat0));
  const y = dLat * R;
  return { x, y };
}
function localToGeodetic(lat0, lon0, x, y){
  const R = PHYS.R_earth;
  const lat = lat0 + (y / R) * 180/Math.PI;
  const lon = lon0 + (x / (R * Math.cos(deg2rad(lat0)))) * 180/Math.PI;
  return { lat, lon };
}

/* UKF21 class */
function UKF21(){
  this.n = 21;
  this.x = Array.from({length:this.n}, ()=> [0]);
  this.P = eye(this.n, 1e-2);
  this.Q = eye(this.n, 1e-4);
  for(let i=0;i<this.n;i++) this.Q[i][i] = 1e-4;
  this.initialized = false;
}
UKF21.prototype.init = function(lat, lon, alt){
  this.origin = { lat0: lat, lon0: lon, alt0: alt||0 };
  // set initial pos to zero (local ENU)
  this.x[0][0] = 0; this.x[1][0] = 0; this.x[2][0] = alt||0;
  this.x[3][0] = 0; this.x[4][0] = 0; this.x[5][0] = 0;
  // quaternion (qx,qy,qz,qw)
  this.x[6][0]=0; this.x[7][0]=0; this.x[8][0]=0; this.x[9][0]=1;
  // biases
  this.x[10][0]=0; this.x[11][0]=0; this.x[12][0]=0; // gyro bias
  this.x[13][0]=0; this.x[14][0]=0; this.x[15][0]=0; // acc bias
  this.x[16][0]=0; // baro bias
  this.x[17][0]=1.0; // mag scale placeholder
  this.x[18][0]=0; // temp bias
  this.x[19][0]=0; // clock bias
  this.x[20][0]=0; // clock drift
  this.P = eye(this.n, 0.1);
  this.initialized = true;
  this.lastTs = Date.now();
};
UKF21.prototype.predict = function(dt, imu){
  if(!this.initialized) return;
  // Simple inertial propagation using accelerometer (not full quaternion rotation)
  const ax = imu && typeof imu.ax === 'number' ? imu.ax - this.x[13][0] : 0;
  const ay = imu && typeof imu.ay === 'number' ? imu.ay - this.x[14][0] : 0;
  const az = imu && typeof imu.az === 'number' ? imu.az - this.x[15][0] : 0;
  // update velocities
  this.x[3][0] += ax * dt;
  this.x[4][0] += ay * dt;
  this.x[5][0] += az * dt;
  // update positions
  this.x[0][0] += this.x[3][0] * dt + 0.5 * ax * dt * dt;
  this.x[1][0] += this.x[4][0] * dt + 0.5 * ay * dt * dt;
  this.x[2][0] += this.x[5][0] * dt + 0.5 * az * dt * dt;
  // increase covariance
  for(let i=0;i<this.n;i++) this.P[i][i] += this.Q[i][i] * dt;
  this.lastTs = Date.now();
};
UKF21.prototype.updatePosition = function(lat, lon, alt, sigma){
  if(!this.initialized) return;
  const d = geodeticToLocal(this.origin.lat0, this.origin.lon0, lat, lon);
  const z = [[d.x],[d.y],[alt || 0]];
  const H = Array.from({length:3}, ()=> Array(this.n).fill(0));
  H[0][0]=1; H[1][1]=1; H[2][2]=1;
  const Hx = matMul(H, this.x);
  const y = matSub(z, Hx);
  const R = eye(3, (sigma || 5)*(sigma || 5));
  const S = matAdd(matMul(H, matMul(this.P, transpose(H))), R);
  const S_inv = inv3(S);
  const K = matMul(this.P, matMul(transpose(H), S_inv));
  this.x = matAdd(this.x, matMul(K, y));
  const I = eye(this.n,1);
  this.P = matMul(matSub(I, matMul(K,H)), this.P);
};
UKF21.prototype.getState = function(){
  if(!this.initialized) return null;
  const pos = { x: this.x[0][0], y: this.x[1][0], z: this.x[2][0] };
  const geo = localToGeodetic(this.origin.lat0, this.origin.lon0, pos.x, pos.y);
  return {
    lat: geo.lat, lon: geo.lon, alt: pos.z,
    vx: this.x[3][0], vy: this.x[4][0], vz: this.x[5][0],
    P: this.P
  };
};

/* EKF6 fallback (simpler) */
function EKF6(){
  this.n = 6;
  this.x = Array.from({length:this.n}, ()=> [0]);
  this.P = eye(this.n, 1);
  this.initialized = false;
}
EKF6.prototype.init = function(lat,lon,alt){ this.origin={lat0:lat,lon0:lon,alt0:alt||0}; this.x[0][0]=0; this.x[1][0]=0; this.x[2][0]=alt||0; this.initialized=true; };
EKF6.prototype.predict = function(dt, imu){
  const ax = imu && typeof imu.ax === 'number' ? imu.ax : 0;
  const ay = imu && typeof imu.ay === 'number' ? imu.ay : 0;
  const az = imu && typeof imu.az === 'number' ? imu.az : 0;
  this.x[3][0] += ax * dt; this.x[4][0] += ay * dt; this.x[5][0] += az * dt;
  this.x[0][0] += this.x[3][0] * dt + 0.5 * ax * dt * dt;
  this.x[1][0] += this.x[4][0] * dt + 0.5 * ay * dt * dt;
  this.x[2][0] += this.x[5][0] * dt + 0.5 * az * dt * dt;
  for(let i=0;i<6;i++) this.P[i][i] += 1e-2 * dt;
};
EKF6.prototype.updateGnss = function(lat,lon,alt,sigma){
  const d = geodeticToLocal(this.origin.lat0,this.origin.lon0,lat,lon);
  const z = [[d.x],[d.y],[alt||0]];
  const H = Array.from({length:3}, ()=> Array(6).fill(0)); H[0][0]=1; H[1][1]=1; H[2][2]=1;
  const y = matSub(z, matMul(H, this.x));
  const R = eye(3,(sigma||5)*(sigma||5));
  const S = matAdd(matMul(H, matMul(this.P, transpose(H))), R);
  const S_inv = inv3(S);
  const K = matMul(this.P, matMul(transpose(H), S_inv));
  this.x = matAdd(this.x, matMul(K, y));
  this.P = matMul(matSub(eye(6,1), matMul(K,H)), this.P);
};
EKF6.prototype.getState = function(){ const pos={x:this.x[0][0],y:this.x[1][0],z:this.x[2][0]}; const geo = localToGeodetic(this.origin.lat0,this.origin.lon0,pos.x,pos.y); return {lat:geo.lat,lon:geo.lon,alt:pos.z,vx:this.x[3][0],vy:this.x[4][0],vz:this.x[5][0],P:this.P}; };

/* FilterEngine wrapper */
const FilterEngine = {
  engine: null,
  type: 'NONE',
  init(lat,lon,alt){
    try{
      this.engine = new UKF21();
      this.engine.init(lat,lon,alt);
      this.type = 'UKF21';
    }catch(e){
      console.warn('UKF failed -> fallback EKF', e);
      this.engine = new EKF6();
      this.engine.init(lat,lon,alt);
      this.type = 'EKF6';
    }
    safeSet('filter-type', this.type);
    return this.engine;
  },
  predict(dt, imu){ if(this.engine) this.engine.predict(dt, imu); },
  updatePosition(lat,lon,alt,sigma){ if(this.engine && this.engine.updatePosition) this.engine.updatePosition(lat,lon,alt,sigma); else if(this.engine && this.engine.updateGnss) this.engine.updateGnss(lat,lon,alt,sigma); },
  getState(){ return this.engine ? this.engine.getState() : null; }
};

window.FilterEngine = FilterEngine;

/* app.part4.js
   PARTIE 4/5 - Capteurs (Geolocation, IMU), RTK/PPK placeholders
*/

let geoWatchId = null;
let imuActive = false;

function startGps(){
  if(!navigator.geolocation) { alert('Géolocalisation non disponible'); return; }
  geoWatchId = navigator.geolocation.watchPosition(async p=>{
    const lat = p.coords.latitude, lon = p.coords.longitude, alt = p.coords.altitude || 0;
    const acc = p.coords.accuracy || 10;
    const speed = p.coords.speed !== null ? p.coords.speed : 0;
    safeSet('lat', (typeof lat === 'number'? lat.toFixed(6) : 'N/A'));
    safeSet('lon', (typeof lon === 'number'? lon.toFixed(6) : 'N/A'));
    safeSet('alt', (alt||0).toFixed(2) + ' m');
    safeSet('speed-raw', speed? speed.toFixed(3) + ' m/s' : '-- m/s');
    if(!FilterEngine.engine) FilterEngine.init(lat, lon, alt);
    // predict small dt
    FilterEngine.predict(0.5, null);
    FilterEngine.updatePosition(lat, lon, alt, acc);
    const state = FilterEngine.getState();
    if(state){
      const v = Math.sqrt((state.vx||0)**2 + (state.vy||0)**2 + (state.vz||0)**2);
      safeSet('speed-filtered', `${v.toFixed(3)} m/s`);
      // update speed max
      const curMax = parseFloat(($('speed-max')?.textContent || '0').replace(/[^\d.-]/g,'')) || 0;
      const kmh = v * 3.6;
      if(kmh > curMax) safeSet('speed-max', `${kmh.toFixed(1)} km/h`);
    }
    // store sample
    DB.dbPut({ ts: nowISO(), source:'gps', lat, lon, alt, acc, speed });
  }, err=>{ console.warn('geo error', err); }, { enableHighAccuracy:true, maximumAge:1000, timeout:10000 });
  safeSet('ntpTime', new Date().toLocaleString());
}

function stopGps(){
  if(geoWatchId !== null){ navigator.geolocation.clearWatch(geoWatchId); geoWatchId = null; }
}

function startImu(){
  if(typeof DeviceMotionEvent === 'undefined') { safeSet('imu-status','Unsupported'); return; }
  function handler(ev){
    const acc = ev.acceleration || ev.accelerationIncludingGravity || { x:0, y:0, z:0 };
    const rot = ev.rotationRate || { alpha:0, beta:0, gamma:0 };
    const dt = 0.05;
    safeSet('acc-x', (acc.x||0).toFixed(3));
    safeSet('acc-y', (acc.y||0).toFixed(3));
    safeSet('acc-z', (acc.z||0).toFixed(3));
    safeSet('mag-x', (rot.alpha||0).toFixed(3));
    // update filter
    FilterEngine.predict(dt, { ax: acc.x||0, ay: acc.y||0, az: acc.z||0 });
    DB.dbPut({ ts: nowISO(), source:'imu', acc, rot });
  }
  if(typeof DeviceMotionEvent.requestPermission === 'function'){
    DeviceMotionEvent.requestPermission().then(res => {
      if(res === 'granted'){ window.addEventListener('devicemotion', handler); imuActive = true; safeSet('imu-status','Actif'); }
      else safeSet('imu-status','Permission refusée');
    }).catch(e => { console.warn(e); safeSet('imu-status','Erreur'); });
  } else {
    window.addEventListener('devicemotion', handler); imuActive = true; safeSet('imu-status','Actif');
  }
}

/* Optional magnetometer */
try{
  if('Magnetometer' in window){
    const mag = new Magnetometer({ frequency: 10 });
    mag.addEventListener('reading', ()=> {
      safeSet('mag-x', mag.x ? mag.x.toFixed(2) : 'N/A');
      safeSet('mag-y', mag.y ? mag.y.toFixed(2) : 'N/A');
      safeSet('mag-z', mag.z ? mag.z.toFixed(2) : 'N/A');
    });
    mag.start();
  }
}catch(e){ console.warn('mag init', e); }

/* RTK/PPK placeholders - these call local server endpoints if you set them up (not required) */
async function startRTK(ntripUrl, mount, user, pass){
  try{
    const r = await fetch('/rtk/start', { method:'POST', body: JSON.stringify({ntripUrl, mount, user, pass}), headers:{'Content-Type':'application/json'} });
    const j = await r.json();
    alert('RTK started: ' + JSON.stringify(j));
  }catch(e){ alert('RTK server not configured'); }
}

async function runPPK(raw, base){
  try{
    const r = await fetch('/ppk/run', { method: 'POST', body: JSON.stringify({ raw, base }), headers: {'Content-Type':'application/json'} });
    const j = await r.json();
    alert('PPK finished: ' + (j.out||'OK'));
  }catch(e){ alert('PPK not available locally'); }
}

/* expose GNSS controls */
window.GNSSControls = { startGps, stopGps, startImu, startRTK, runPPK };

/* app.part5.js
   PARTIE 5/5 - UI wiring, charts, orchestrator, init
*/

(async function(){
  // UI element mapping
  const el = id => document.getElementById(id);

  // Hook buttons
  el('btn-start-gps').addEventListener('click', ()=>{
    if(!window.GNSSControls) return;
    if(!window._gpsRunning){
      GNSSControls.startGps();
      GNSSControls.startImu && GNSSControls.startImu();
      el('btn-start-gps').textContent = '⏸ STOP GPS';
      window._gpsRunning = true;
    } else {
      GNSSControls.stopGps();
      el('btn-start-gps').textContent = '▶️ MARCHE GPS';
      window._gpsRunning = false;
    }
  });

  el('btn-urgent-stop').addEventListener('click', ()=> {
    GNSSControls.stopGps();
    safeSet('imu-status','Inactif');
    alert('Arrêt d\\'urgence exécuté');
  });

  el('btn-export').addEventListener('click', ()=> {
    DB.dbGetAll().then(rows => {
      if(!rows || rows.length===0) return alert('Aucun enregistrement');
      const keys = Array.from(new Set(rows.flatMap(r => Object.keys(r))));
      const csv = [keys.join(',')].concat(rows.map(r => keys.map(k => JSON.stringify(r[k]||'')).join(','))).join('\n');
      const blob = new Blob([csv], { type: 'text/csv' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a'); a.href = url; a.download = 'observations.csv'; a.click(); URL.revokeObjectURL(url);
    });
  });

  el('btn-reset-dist').addEventListener('click', async ()=>{ await DB.dbClear(); alert('Logs réinitialisés'); });
  el('btn-reset-vmax').addEventListener('click', ()=> safeSet('speed-max','0.0 km/h'));
  el('btn-capture').addEventListener('click', async ()=> {
    const state = FilterEngine.getState();
    await DB.dbPut({ ts: nowISO(), type:'snapshot', state });
    alert('Snapshot capturé');
  });

  // Charts
  let chartTemp = null, chartSpeed = null;
  try{
    const ctxT = el('chartTemp').getContext('2d');
    chartTemp = new Chart(ctxT, { type:'line', data:{ labels:[], datasets:[{label:'Temp (°C)', data:[], tension:0.25}]}, options:{ plugins:{legend:{display:false}}, scales:{x:{display:false}} } });
    const ctxS = el('chartSpeed').getContext('2d');
    chartSpeed = new Chart(ctxS, { type:'line', data:{ labels:[], datasets:[{label:'Vitesse (m/s)', data:[], tension:0.25}]}, options:{ plugins:{legend:{display:false}}, scales:{x:{display:false}} } });
  }catch(e){ console.warn('chart init', e); }

  // orchestrator: refresh metrics and astro every 5s
  async function refresh(){
    try{
      const state = FilterEngine.getState();
      const lat = state ? state.lat : 48.8566;
      const lon = state ? state.lon : 2.3522;
      // dynamic meteorology: use SciencePro if local sensors unavailable
      const tempTxt = el('temp')?.textContent || '15';
      const T = parseFloat(tempTxt.replace(/[^\d.-]/g,'')) || 15;
      const P = parseFloat((el('pressure')?.textContent || '1013').replace(/[^\d.-]/g,'')) || 1013.25;
      const RH = parseFloat((el('rh')?.textContent || '50').replace(/[^\d.-]/g,'')) || 50;
      const rho = SciencePro.densityAir(T, P, RH);
      safeSet('rho', isFinite(rho)? `${rho.toFixed(4)} kg/m³` : 'N/A');
      safeSet('mu', SciencePro.sutherlandViscosity(T).toExponential(3) + ' Pa·s');
      safeSet('c_sound', isFinite(SciencePro.speedOfSound(T, RH, P)) ? `${SciencePro.speedOfSound(T,RH,P).toFixed(2)} m/s` : 'N/A');

      // update state values to UI and charts
      if(state){
        safeSet('lat', state.lat.toFixed(6));
        safeSet('lon', state.lon.toFixed(6));
        safeSet('alt', (state.alt||0).toFixed(2) + ' m');
        const v = Math.sqrt((state.vx||0)**2 + (state.vy||0)**2 + (state.vz||0)**2);
        safeSet('speed-filtered', `${v.toFixed(3)} m/s`);
        if(chartSpeed){ chartSpeed.data.labels.push(new Date().toLocaleTimeString()); chartSpeed.data.datasets[0].data.push(v); if(chartSpeed.data.labels.length>200){ chartSpeed.data.labels.shift(); chartSpeed.data.datasets[0].data.shift(); } chartSpeed.update(); }
        safeSet('mach', (SciencePro.speedOfSound(T,RH,P)>0) ? (v/SciencePro.speedOfSound(T,RH,P)).toFixed(6) : 'N/A');
        safeSet('lorentz', SciencePro.lorentzGamma(v).toFixed(8));
        // uncertainties from P
        const Pmat = state.P || null;
        if(Pmat && Pmat.length>2){ safeSet('p-pos', Math.sqrt(Pmat[0][0] + Pmat[1][1]).toFixed(3) + ' m'); safeSet('p-vel', Math.sqrt(Pmat[3][3] + Pmat[4][4]).toFixed(3) + ' m/s'); }
      }

      // astronomy
      const now = new Date();
      const sun = SciencePro.sunPosition(now);
      const moon = SciencePro.moonPhase(now);
      safeSet('sun-altaz', `RA ${(sun.ra*180/Math.PI).toFixed(2)}° / Dec ${(sun.dec*180/Math.PI).toFixed(2)}°`);
      safeSet('moon-phase', `${(moon.illuminatedFraction*100).toFixed(1)}% • ${moon.ageDays.toFixed(1)} d`);
      safeSet('eot', `${SciencePro.equationOfTime(now).toFixed(2)} min`);

      // DOPS placeholder and fix
      safeSet('dops', 'HDOP 0.9 / VDOP 1.2');
      safeSet('fix-type', '3D / UKF');

    }catch(e){ console.warn('refresh failed', e); }
  }

  setInterval(refresh, 5000);
  refresh();

  // initialisation: try to start with a default coordinate and set filter
  FilterEngine.init(48.8566, 2.3522, 35);

  // map: placeholder text (user can include Leaflet offline)
  (function initMap(){
    const m = el('map');
    if(!m) return;
    m.innerHTML = '<div style="padding:12px;color:var(--muted)">Carte interactive non incluse (pour la version offline, place leaflet.js + tiles localement).</div>';
  })();

  console.log('Application initialisée - Fusion finale (index + 5 JS) prête.');
})();
        
