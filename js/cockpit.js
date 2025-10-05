// Carte Leaflet
var map = L.map('map').setView([48.8566,2.3522], 13);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
var marker = L.marker([48.8566,2.3522]).addTo(map);

// Variables simulation
var startTime = Date.now(), totalDistance=0, speedMax=0, lat=48.8566, lon=2.3522;

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

// Mise à jour dynamique
function updateData(){
    var now = Date.now();
    var elapsed = (now - startTime)/1000;
    document.getElementById('time').textContent = elapsed.toFixed(2);

    // GPS simulation
    lat += (Math.random()-0.5)/1000;
    lon += (Math.random()-0.5)/1000;
    marker.setLatLng([lat,lon]);
    document.getElementById('lat').textContent = lat.toFixed(6);
    document.getElementById('lon').textContent = lon.toFixed(6);
    var alt = 30 + Math.random()*10;
    document.getElementById('alt').textContent = alt.toFixed(1);
    document.getElementById('gpsAcc').textContent = (90+Math.random()*10).toFixed(0);

    // Vitesse
    var speed = Math.random()*50;
    document.getElementById('speed').textContent = speed.toFixed(1);
    speedMax = Math.max(speedMax, speed);
    document.getElementById('speedMax').textContent = speedMax.toFixed(1);
    totalDistance += speed*1000/3600;
    document.getElementById('distanceKm').textContent = (totalDistance/1000).toFixed(3);
    document.getElementById('distanceM').textContent = totalDistance.toFixed(1);
    document.getElementById('distanceMm').textContent = (totalDistance*1000).toFixed(0);
    document.getElementById('distanceLightS').textContent = (totalDistance/299792458).toFixed(6);
    document.getElementById('distanceAL').textContent = (totalDistance/9.461e15).toExponential(3);

    // IMU simulation
    document.getElementById('ax').textContent = (Math.random()*2-1).toFixed(2);
    document.getElementById('ay').textContent = (Math.random()*2-1).toFixed(2);
    document.getElementById('az').textContent = (9.8+(Math.random()-0.5)).toFixed(2);
    document.getElementById('gx').textContent = (Math.random()*2-1).toFixed(2);
    document.getElementById('gy').textContent = (Math.random()*2-1).toFixed(2);
    document.getElementById('gz').textContent = (Math.random()*2-1).toFixed(2);
    document.getElementById('heading').textContent = Math.floor(Math.random()*360);
    document.getElementById('mx').textContent = (Math.random()*50-25).toFixed(1);
    document.getElementById('my').textContent = (Math.random()*50-25).toFixed(1);
    document.getElementById('mz').textContent = (Math.random()*50-25).toFixed(1);

    // Météo simulation
    document.getElementById('temp').textContent = (20+Math.random()*5).toFixed(1);
    document.getElementById('pressure').textContent = (1010+Math.random()*5).toFixed(1);
    document.getElementById('hum').textContent = (40+Math.random()*20).toFixed(0);
    document.getElementById('wind').textContent = (Math.random()*20).toFixed(0);
    document.getElementById('clouds').textContent = (Math.random()*100).toFixed(0);
    document.getElementById('rain').textContent = (Math.random()*5).toFixed(1);
    document.getElementById('snow').textContent = (Math.random()*2).toFixed(1);
    document.getElementById('uv').textContent = (Math.random()*12).toFixed(1);
    document.getElementById('airQuality').textContent = Math.floor(Math.random()*500);
    document.getElementById('boilPoint').textContent = (100+Math.random()*5).toFixed(1);

    // Soleil & Lune simulation
    document.getElementById('sunCul').textContent = '12:30';
    document.getElementById('trueSolar').textContent = '12:28';
    document.getElementById('meanSolar').textContent = '12:30';
    document.getElementById('eqTime').textContent = '+/- 1.5 min';
    document.getElementById('moonPhase').textContent = (Math.random()*100).toFixed(0);
    document.getElementById('moonMag').textContent = (Math.random()*1).toFixed(2);
    document.getElementById('moonRise').textContent = '20:15';
    document.getElementById('moonSet').textContent = '06:05';
    document.getElementById('moonCul').textContent = '01:45';

    // Capteurs supplémentaires
    document.getElementById('level').textContent = (Math.random()*5-2.5).toFixed(2);
    document.getElementById('lux').textContent = (Math.random()*1000).toFixed(0);
    document.getElementById('dB').textContent = (30+Math.random()*50).toFixed(1);
    document.getElementById('freq').textContent = (Math.random()*2000).toFixed(0);

    mcClock();
    drawSismograph();
    drawAttitude();
}
setInterval(updateData,1000);

// Sismogramme
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

// Attitude IMU
var attCanvas = document.getElementById('attitude');
var attCtx = attCanvas.getContext('2d');
function drawAttitude(){
    attCtx.clearRect(0,0,attCanvas.width,attCanvas.height);
    let pitch = parseFloat(document.getElementById('ax').textContent);
    let roll = parseFloat(document.getElementById('ay').textContent);
    attCtx.fillStyle='blue';
    attCtx.fillRect(attCanvas.width/2 + roll*5, attCanvas.height/2 + pitch*5, 50, 5);
      }
                                                 
