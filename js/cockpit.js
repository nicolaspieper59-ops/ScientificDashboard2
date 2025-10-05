// Carte Leaflet
var map = L.map('map').setView([48.8566,2.3522], 13);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
var marker = L.marker([48.8566,2.3522]).addTo(map);

let running = true;
document.getElementById('toggleBtn').addEventListener('click', () => {
    running = !running;
    document.getElementById('toggleBtn').textContent = running ? 'Arrêt' : 'Marche';
});

let startTime = Date.now();
let prevLat = null, prevLon = null, totalDistance = 0, speedMax = 0;

// Fonction centrale pour mise à jour de tous les capteurs
function updateAllSensors(){
    if(!running) return;

    // Temps écoulé
    let now = Date.now();
    document.getElementById('time').textContent = ((now-startTime)/1000).toFixed(2);

    // === GPS réel ===
    if(navigator.geolocation){
        navigator.geolocation.getCurrentPosition(pos => {
            let lat = pos.coords.latitude;
            let lon = pos.coords.longitude;
            let alt = pos.coords.altitude || 0;
            let acc = pos.coords.accuracy;

            document.getElementById('lat').textContent = lat.toFixed(6);
            document.getElementById('lon').textContent = lon.toFixed(6);
            document.getElementById('alt').textContent = alt.toFixed(1);
            document.getElementById('gpsAcc').textContent = acc.toFixed(1);

            marker.setLatLng([lat, lon]);

            // Calcul distance et vitesse
            if(prevLat!==null){
                let R = 6371000; // rayon Terre en m
                let dLat = (lat-prevLat)*Math.PI/180;
                let dLon = (lon-prevLon)*Math.PI/180;
                let a = Math.sin(dLat/2)**2 + Math.cos(prevLat*Math.PI/180)*Math.cos(lat*Math.PI/180)*Math.sin(dLon/2)**2;
                let c = 2*Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
                let dist = R*c;
                totalDistance += dist;
                document.getElementById('distanceM').textContent = totalDistance.toFixed(1);
                document.getElementById('distanceKm').textContent = (totalDistance/1000).toFixed(3);
                document.getElementById('distanceMm').textContent = (totalDistance*1000).toFixed(0);

                let speed = dist; // approximatif m/s par seconde
                speedMax = Math.max(speedMax, speed);
                document.getElementById('speed').textContent = (speed*3.6).toFixed(1);
                document.getElementById('speedMax').textContent = (speedMax*3.6).toFixed(1);
            }
            prevLat = lat;
            prevLon = lon;
        });
    }

    // === IMU réel (DeviceMotion / DeviceOrientation) ===
    window.addEventListener('devicemotion', function(e){
        let a = e.accelerationIncludingGravity;
        document.getElementById('ax').textContent = a.x?.toFixed(2) ?? '—';
        document.getElementById('ay').textContent = a.y?.toFixed(2) ?? '—';
        document.getElementById('az').textContent = a.z?.toFixed(2) ?? '—';
    });

    window.addEventListener('deviceorientation', function(e){
        document.getElementById('gx').textContent = e.alpha?.toFixed(2) ?? '—';
        document.getElementById('gy').textContent = e.beta?.toFixed(2) ?? '—';
        document.getElementById('gz').textContent = e.gamma?.toFixed(2) ?? '—';
        document.getElementById('heading').textContent = e.alpha?.toFixed(0) ?? '—';
    });

    // === Lumière et son via capteurs internes ===
    if('AmbientLightSensor' in window){
        let sensor = new AmbientLightSensor();
        sensor.addEventListener('reading', () => document.getElementById('lux').textContent = sensor.illuminance.toFixed(0));
        sensor.start();
    }

    if('navigator' in window && navigator.mediaDevices){
        navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
            let context = new AudioContext();
            let source = context.createMediaStreamSource(stream);
            let analyser = context.createAnalyser();
            source.connect(analyser);
            analyser.fftSize = 256;
            let dataArray = new Uint8Array(analyser.frequencyBinCount);
            function updateSound(){
                analyser.getByteFrequencyData(dataArray);
                let avg = dataArray.reduce((a,b)=>a+b)/dataArray.length;
                document.getElementById('dB').textContent = avg.toFixed(1);
                requestAnimationFrame(updateSound);
            }
            updateSound();
        }).catch(()=>{ document.getElementById('dB').textContent='—'; });
    }

    // === Météo via API (ex: OpenWeatherMap) ===
    // Remplacer YOUR_API_KEY par ta clé
    let latCenter = prevLat ?? 48.8566;
    let lonCenter = prevLon ?? 2.3522;
    fetch(`https://api.openweathermap.org/data/2.5/weather?lat=${latCenter}&lon=${lonCenter}&units=metric&appid=YOUR_API_KEY`)
        .then(res=>res.json())
        .then(data=>{
            document.getElementById('temp').textContent = data.main.temp.toFixed(1);
            document.getElementById('pressure').textContent = data.main.pressure;
            document.getElementById('hum').textContent = data.main.humidity;
            document.getElementById('wind').textContent = data.wind.speed;
        }).catch(()=>{});

    mcClock();
    drawSismograph();
    drawAttitude();
}

setInterval(updateAllSensors, 1000);

// === Fonctions Canvas (sismogramme & attitude) ===
var seisCanvas = document.getElementById('seis');
var seisCtx = seisCanvas.getContext('2d');
function drawSismograph(){
    seisCtx.clearRect(0,0,seisCanvas.width,seisCanvas.height);
    seisCtx.beginPath();
    for(let i=0;i<seisCanvas.width;i+=2){
        let y = seisCanvas.height/2 + Math.sin(Date.now()/500 + i/10)*30;
        if(i===0) seisCtx.moveTo(i,y); else seisCtx.lineTo(i,y);
    }
    seisCtx.strokeStyle='red';
    seisCtx.stroke();
}

var attCanvas = document.getElementById('attitude');
var attCtx = attCanvas.getContext('2d');
function drawAttitude(){
    attCtx.clearRect(0,0,attCanvas.width,attCanvas.height);
    let pitch = parseFloat(document.getElementById('ax').textContent) || 0;
    let roll = parseFloat(document.getElementById('ay').textContent) || 0;
    attCtx.fillStyle='blue';
    attCtx.fillRect(attCanvas.width/2 + roll*5, attCanvas.height/2 + pitch*5, 50, 5);
}

// Horloge Minecraft
function mcClock(){
    let now = new Date();
    let ticks = Math.floor(now.getTime()/50);
    let hours = Math.floor(ticks/1000)%24;
    let minutes = Math.floor((ticks%1000)/1000*60);
    let seconds = Math.floor(((ticks%1000)/1000*60)%60);
    document.getElementById('mcClock').textContent =
      `${hours.toString().padStart(2,'0')}:${minutes.toString().padStart(2,'0')}:${seconds.toString().padStart(2,'0')}`;
        }
            
