/* cockpit.js — version ultime (module-like but plain script loaded with type=module in HTML) */
/* Depends: Leaflet (loaded in HTML), SunCalc (loaded in HTML) */

let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let vitesseMax = 0;
let distanceTotale = 0;
let t0 = null;

let destination = null;
let map = null, marker = null, destMarker = null;

const set = (id, txt) => {
  const el = document.getElementById(id);
  if (el) el.textContent = txt;
};
const setHtml = (id, html) => {
  const el = document.getElementById(id);
  if (el) el.innerHTML = html;
};
const saveLocal = (k, v) => { try { localStorage.setItem(k, JSON.stringify(v)); } catch(e){} };
const loadLocal = (k) => { try { return JSON.parse(localStorage.getItem(k)); } catch(e){ return null } };

// UI elements
const btnStart = document.getElementById('marche');
const btnStop = document.getElementById('stop');
const btnReset = document.getElementById('reset');
const inputMasse = document.getElementById('masse');
const btnToggleRituel = document.getElementById('toggle-rituel');
const chkPhys = document.getElementById('toggle-physique');
const chkChim = document.getElementById('toggle-chimie');
const chkSvt = document.getElementById('toggle-svt');
const medaillonZoom = document.getElementById('medaillon-zoom');

// base placeholders to avoid infinite concatenation
const capteursBase = {
  lumiere: '--',
  son: '--',
  hz: '--',
  niveau: '--',
  magnetisme: '--'
};

function restorePlaceholders() {
  set('capteurs', `Lumière : ${capteursBase.lumiere} lux | Son : ${capteursBase.son} dB | Fréquence : ${capteursBase.hz} Hz | Niveau : ${capteursBase.niveau}° | Magnétisme : ${capteursBase.magnetisme} µT`);
}
restorePlaceholders();

// Chrono
function startChrono(){
  t0 = performance.now();
  const el = document.getElementById('temps');
  (function tick(){
    if(!t0) return;
    const s = (performance.now() - t0)/1000;
    el.textContent = `Temps : ${s.toFixed(2)} s`;
    requestAnimationFrame(tick);
  })();
}

// GPS
function startGPS(){
  if (!('geolocation' in navigator)) {
    set('gps','GPS non disponible (HTTPS requis)');
    return;
  }
  if (watchId !== null) return;
  t0 = performance.now();
  navigator.geolocation.getCurrentPosition(
    p => traiterPosition(p.coords, p.timestamp),
    e => set('gps', 'Erreur GPS: ' + (e.message || e.code)),
    { enableHighAccuracy:true, timeout:10000 }
  );
  watchId = navigator.geolocation.watchPosition(
    p => traiterPosition(p.coords, p.timestamp),
    e => set('gps', 'Erreur GPS: ' + (e.message || e.code)),
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );
  startChrono();
}

function stopGPS(){
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    set('gps','⏹️ Suivi arrêté');
  }
}

function resetAll(){
  vitesses = []; vitesseMax = 0; distanceTotale = 0; positionPrecedente = null; t0 = null;
  restorePlaceholders();
  ['vitesse','vitesse-moy','vitesse-max','vitesse-ms','pourcentage','distance','distance-cosmique','gps','meteo','qualite-air','grandeurs'].forEach(id=>set(id,'--'));
  // reset map markers
  if (destMarker) { destMarker.remove(); destMarker = null; destination = null; set('destination','Destination : Aucune'); }
}

// Position processing
function traiterPosition(coords, timestamp){
  // coords: {latitude, longitude, altitude, speed, accuracy, heading}
  // compute speed (km/h) - prefer coords.speed if available
  const v = (typeof coords.speed === 'number' && coords.speed !== null) ? coords.speed * 3.6 : calculerVitesse(coords, timestamp);
  if (v >= 0 && v < 10000) {
    vitesseMax = Math.max(vitesseMax, v);
    vitesses.push(v);
    const moyenne = vitesses.length ? (vitesses.reduce((a,b)=>a+b,0)/vitesses.length) : 0;
    const mps = v/3.6;
    const mmps = mps*1000;
    set('vitesse', `Vitesse instantanée : ${v.toFixed(2)} km/h`);
    set('vitesse-moy', `Vitesse moyenne : ${moyenne.toFixed(2)} km/h`);
    set('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
    set('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
    set('pourcentage', `% Lumière : ${(mps/299792458*100).toExponential(2)}% | % Son : ${(mps/343*100).toFixed(2)}%`);
    set('distance', `Distance : ${(distanceTotale/1000).toFixed(3)} km | ${distanceTotale.toFixed(1)} m`);
    const ds = distanceTotale/299792458;
    const dal = ds/(3600*24*365.25);
    set('distance-cosmique', `Distance cosmique : ${ds.toFixed(6)} s lumière | ${dal.toExponential(3)} al`);
    set('gps', `Lat:${coords.latitude.toFixed(6)} Lon:${coords.longitude.toFixed(6)} Alt:${(coords.altitude==null?'--':coords.altitude.toFixed(1))}m Acc:${(coords.accuracy||'--')}`);
    // map
    if (!map) initMap(coords.latitude, coords.longitude);
    if (marker) marker.setLatLng([coords.latitude, coords.longitude]);
    if (map) map.setView([coords.latitude, coords.longitude]);
    // medaillon & astro
    afficherMedaillon(coords.latitude, coords.longitude);
    // meteo + air
    chargerMeteo(coords.latitude, coords.longitude);
    // grandeurs
    chargerGrandeurs();
    // save last
    try { saveLocal('cockpit.lastGPS', {lat:coords.latitude, lon:coords.longitude, alt:coords.altitude, t:timestamp}); } catch(e){}
    positionPrecedente = {...coords, timestamp};
  }
}

function calculerVitesse(coords, t){
  if (!positionPrecedente) {
    positionPrecedente = {...coords, timestamp:t};
    return 0;
  }
  const dt = (t - positionPrecedente.timestamp)/1000;
  const d = calculerDistance(coords, positionPrecedente);
  if (Number.isFinite(d) && d>=0) distanceTotale += d;
  positionPrecedente = {...coords, timestamp:t};
  return dt>0 ? (d/dt)*3.6 : 0;
}

function calculerDistance(a,b){
  const R = 6371e3;
  const φ1 = a.latitude*Math.PI/180;
  const φ2 = b.latitude*Math.PI/180;
  const Δφ = (b.latitude - a.latitude)*Math.PI/180;
  const Δλ = (b.longitude - a.longitude)*Math.PI/180;
  const A = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const C = 2 * Math.atan2(Math.sqrt(A), Math.sqrt(1-A));
  return R * C;
}

/* ---------- Carte (Leaflet) ---------- */
function initMap(lat, lon){
  try {
    map = L.map('map').setView([lat,lon], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{
      attribution: '© OpenStreetMap contributors'
    }).addTo(map);
    marker = L.marker([lat,lon]).addTo(map).bindPopup('Vous êtes ici');
    map.on('click', e=>{
      destination = e.latlng;
      if (destMarker) destMarker.remove();
      destMarker = L.marker(destination).addTo(map).bindPopup('Destination').openPopup();
      set('destination', `Destination : ${destination.lat.toFixed(6)}, ${destination.lng.toFixed(6)}`);
    });
  } catch (e) {
    console.warn('Leaflet non disponible', e);
  }
}

document.getElementById('use-current-as-dest')?.addEventListener('click', ()=>{
  if (!positionPrecedente) return;
  const lat = positionPrecedente.latitude, lon = positionPrecedente.longitude;
  destination = {lat, lng:lon};
  if (!map) initMap(lat, lon);
  if (destMarker) destMarker.remove();
  destMarker = L.marker([lat,lon]).addTo(map).bindPopup('Destination (moi)').openPopup();
  set('destination', `Destination : ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
});
document.getElementById('clear-dest')?.addEventListener('click', ()=>{
  if(destMarker) { destMarker.remove(); destMarker = null; destination = null; set('destination','Destination : Aucune'); }
});

/* ---------- Médaillon (canvas + astro) ---------- */
function afficherMedaillon(lat = 43.3, lon = 5.4) {
  const now = new Date();
  const med = document.getElementById('medaillon');
  const zoom = parseFloat(medaillonZoom?.value || 1);
  // create canvas if needed
  let canvas = med.querySelector('canvas');
  if (!canvas) {
    canvas = document.createElement('canvas');
    canvas.width = 320; canvas.height = 320;
    med.appendChild(canvas);
  }
  const ctx = canvas.getContext('2d');
  // background
  ctx.fillStyle = '#000';
  ctx.fillRect(0,0,canvas.width,canvas.height);
  const cx = canvas.width/2, cy = canvas.height/2;
  const r = 120 * zoom;
  // horizon circle
  ctx.strokeStyle = '#444'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.stroke();
  // sun/moon positions via SunCalc
  try {
    const times = SunCalc.getTimes(now, lat, lon);
    const noon = times.solarNoon;
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    // convert azimuth/alt to canvas coords (simple projection)
    const azToX = (az, alt) => {
      // az in rad, alt in rad -> x,y
      const dist = (90 - (alt * 180/Math.PI))/90 * r; // rudimentary
      const x = cx + dist * Math.sin(az);
      const y = cy - dist * Math.cos(az);
      return {x,y};
    };
    const sunC = azToX(sunPos.azimuth, sunPos.altitude);
    ctx.fillStyle = '#ffd700'; ctx.beginPath(); ctx.arc(sunC.x, sunC.y, 8*zoom,0,2*Math.PI); ctx.fill();
    ctx.fillStyle = '#fff'; ctx.font = `${12*zoom}px monospace`; ctx.fillText('☀️ Soleil', sunC.x+10*zoom, sunC.y);
    const moonC = azToX(moonPos.azimuth, moonPos.altitude);
    ctx.fillStyle = '#ccc'; ctx.beginPath(); ctx.arc(moonC.x, moonC.y, 6*zoom,0,2*Math.PI); ctx.fill();
    ctx.fillText('🌙 Lune', moonC.x-40*zoom, moonC.y-6*zoom);
    // stars placeholders
    ctx.fillStyle = '#0ff';
    const stars = [{x:cx+20,y:cy+60,name:'Orion'},{x:cx-30,y:cy+40,name:'Cassiopeia'},{x:cx+70,y:cy+10,name:'Scorpius'}];
    stars.forEach(s => { ctx.beginPath(); ctx.arc(s.x,s.y,2*zoom,0,2*Math.PI); ctx.fill(); ctx.fillText(s.name, s.x+4*zoom, s.y+6*zoom); });
    // small galaxy
    ctx.fillStyle = '#f0f'; ctx.beginPath(); ctx.arc(cx, cy+100*zoom, 3*zoom, 0,2*Math.PI); ctx.fill(); ctx.fillText('🌌 Galaxie', cx-20*zoom, cy+110*zoom);
  } catch(e){ console.warn('SunCalc error', e); }
  // update astro textual fields
  try {
    const times = SunCalc.getTimes(new Date(), lat, lon);
    set('culmination-soleil', `Culmination soleil : ${times.solarNoon?.toLocaleTimeString() ?? '--'}`);
    set('heure-vraie', `Lever soleil : ${times.sunrise?.toLocaleTimeString() ?? '--'}`);
    set('heure-moyenne', `Coucher soleil : ${times.sunset?.toLocaleTimeString() ?? '--'}`);
    const eq = ((times.sunset - times.sunrise)/60000 - 720).toFixed(2);
    set('equation-temps', `Équation du temps : ${eq} min`);
    const moonTimes = SunCalc.getMoonTimes(new Date(), lat, lon);
    const moonIll = SunCalc.getMoonIllumination(new Date());
    set('lune-phase', `Lune phase : ${(moonIll.fraction*100).toFixed(2)}%`);
    set('lune-mag', `Illum : ${(moonIll.phase).toFixed(2)}`);
    set('lune-lever', `Lever lune : ${moonTimes.rise?.toLocaleTimeString() ?? '--'}`);
    set('lune-coucher', `Coucher lune : ${moonTimes.set?.toLocaleTimeString() ?? '--'}`);
    set('lune-culmination', moonTimes.alwaysUp ? 'Toujours visible' : moonTimes.alwaysDown ? 'Toujours sous l horizon' : '--');
  } catch(e){}
}

/* ---------- Capteurs (safe, non-duplicating updates) ---------- */
let audioAnalyser = null;
function activerCapteurs(){
  // reset displayed base
  restorePlaceholders();

  // AmbientLightSensor
  if ('AmbientLightSensor' in window) {
    try {
      const sensor = new AmbientLightSensor();
      sensor.addEventListener('reading', () => {
        capteursBase.lumiere = Math.round(sensor.illuminance);
        updateCapteursDisplay();
      });
      sensor.start();
    } catch(e){ console.warn('AmbientLightSensor start failed', e); }
  }

  // Microphone analyser (update sound only)
  if (navigator.mediaDevices?.getUserMedia) {
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      try {
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        audioAnalyser = ctx.createAnalyser();
        audioAnalyser.fftSize = 2048;
        const source = ctx.createMediaStreamSource(stream);
        source.connect(audioAnalyser);
        const data = new Uint8Array(audioAnalyser.frequencyBinCount);

        function sampleAudio(){
          audioAnalyser.getByteFrequencyData(data);
          const avg = data.reduce((a,b)=>a+b,0)/data.length || 0;
          const dB = 20 * Math.log10(avg || 1);
          capteursBase.son = Math.round(dB);
          capteursBase.hz = audioAnalyser.frequencyBinCount;
          updateCapteursDisplay();
          requestAnimationFrame(sampleAudio);
        }
        sampleAudio();
      } catch(e){ console.warn('Audio analyser error', e); }
    }).catch(err=>console.warn('getUserMedia denied or failed', err));
  }

  // DeviceOrientation for level
  if ('DeviceOrientationEvent' in window) {
    window.addEventListener('deviceorientation', e=>{
      if (typeof e.beta === 'number') {
        capteursBase.niveau = e.beta.toFixed(1);
        updateCapteursDisplay();
      }
    });
  }

  // Magnetometer
  if ('Magnetometer' in window) {
    try {
      const mag = new Magnetometer();
      mag.addEventListener('reading', ()=>{
        const champ = Math.sqrt(mag.x**2 + mag.y**2 + mag.z**2);
        capteursBase.magnetisme = Math.round(champ);
        updateCapteursDisplay();
      });
      mag.start();
    } catch(e){ console.warn('Magnetometer failed', e); }
  }
}

function updateCapteursDisplay(){
  set('capteurs', `Lumière : ${capteursBase.lumiere} lux | Son : ${capteursBase.son} dB | Fréquence : ${capteursBase.hz} Hz | Niveau : ${capteursBase.niveau}° | Magnétisme : ${capteursBase.magnetisme} µT`);
}

/* ---------- Météo & Qualité d'air ---------- */
async function chargerMeteo(lat, lon){
  if (!lat || !lon) {
    const last = loadLocal('cockpit.lastGPS');
    if (last) { lat = last.lat; lon = last.lon; }
    else { lat = 43.3; lon = 5.4; }
  }
  // Open-Meteo current_weather
  try {
    const res = await fetch(`https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}&current_weather=true`);
    const j = await res.json();
    const w = j.current_weather;
    set('meteo', `Temp : ${w.temperature} °C | Vent : ${w.windspeed} km/h | Code : ${w.weathercode}`);
    saveLocal('cockpit.meteo', w);
  } catch(e){
    const cached = loadLocal('cockpit.meteo');
    if (cached) set('meteo', `(offline) Temp : ${cached.temperature} °C | Vent : ${cached.windspeed} km/h`);
    else set('meteo','Météo indisponible');
  }
  // OpenAQ
  try {
    const res2 = await fetch(`https://api.openaq.org/v2/latest?coordinates=${lat},${lon}`);
    const j2 = await res2.json();
    const pm25 = j2.results?.[0]?.measurements?.find(m=>m.parameter==='pm25')?.value ?? '--';
    set('qualite-air', `PM2.5 : ${pm25} µg/m³`);
    saveLocal('cockpit.air', pm25);
  } catch(e){
    const cached = loadLocal('cockpit.air');
    set('qualite-air', cached ? `(offline) PM2.5 : ${cached} µg/m³` : 'Qualité air indisponible');
  }
}

/* ---------- Grandeurs physiques / chimiques / biologiques ---------- */
function chargerGrandeurs(){
  const masse = parseFloat(inputMasse?.value) || 70;
  const v_kmh = vitesses.length ? vitesses[vitesses.length-1] : 0;
  const v = v_kmh / 3.6;
  const g = 9.81;
  let out = '';
  if (chkPhys?.checked) {
    const F = masse * g;
    const E = 0.5 * masse * v * v;
    const P = F * v;
    out += `🧲 Physique — Force : ${F.toFixed(1)} N | Énergie cinétique : ${E.toFixed(1)} J | Puissance estimée : ${P.toFixed(1)} W\n`;
  }
  if (chkChim?.checked) {
    const R = 8.314, T = 298;
    const Vm = (R * T / 101325).toFixed(4);
    out += `🧪 Chimie — Volume molaire (approx) : ${Vm} m³/mol | Avogadro : 6.022e23\n`;
  }
  if (chkSvt?.checked) {
    const bpm = Math.round(60 + v * 2);
    const temp = (36.5 + v * 0.02).toFixed(1);
    out += `🌱 SVT — Fréquence cardiaque simulée : ${bpm} bpm | Température : ${temp} °C\n`;
  }
  set('grandeurs', out);
}

/* ---------- Horloges ---------- */
function activerHorloge(){
  const el = document.getElementById('horloge-minecraft');
  (function loop(){
    const d = new Date();
    el.textContent = `${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;
    requestAnimationFrame(loop);
  })();
}
function synchroniserHeureAtomique(){
  fetch('https://worldtimeapi.org/api/ip').then(r=>r.json()).then(j=>{
    const utc = new Date(j.utc_datetime);
    set('horloge-atomique', `Heure atomique (UTC) : ${utc.toLocaleTimeString()}`);
  }).catch(()=>set('horloge-atomique','Heure atomique indisponible'));
}

/* ---------- Toggle rituel / UI ---------- */
btnToggleRituel?.addEventListener('click', ()=>{
  const on = document.body.classList.toggle('rituel-off');
  btnToggleRituel.textContent = on ? '🔋 Rituel cosmique : OFF' : '✨ Rituel cosmique : ON';
});

/* ---------- Binding buttons ---------- */
btnStart?.addEventListener('click', ()=>{
  startGPS(); activerCapteurs(); activerHorloge(); synchroniserHeureAtomique();
  afficherMedaillon(); chargerGrandeurs();
});
btnStop?.addEventListener('click', stopGPS);
btnReset?.addEventListener('click', ()=>{ resetAll(); });

// zoom medaillon redraw
medaillonZoom?.addEventListener('input', ()=> {
  // redraw medaillon with last known coords or defaults
  const last = loadLocal('cockpit.lastGPS');
  if (last) afficherMedaillon(last.lat, last.lon);
  else afficherMedaillon();
});

// initial restore of cached values
(function initFromCache(){
  const lastGPS = loadLocal('cockpit.lastGPS');
  const lastMeteo = loadLocal('cockpit.meteo');
  const lastAir = loadLocal('cockpit.air');
  if (lastGPS) { set('gps', `Lat:${lastGPS.lat.toFixed(4)} Lon:${lastGPS.lon.toFixed(4)}`); }
  if (lastMeteo) set('meteo', `(cached) Temp : ${lastMeteo.temperature} °C`);
  if (lastAir) set('qualite-air', `(cached) PM2.5 : ${lastAir} µg/m³`);
})();
  
