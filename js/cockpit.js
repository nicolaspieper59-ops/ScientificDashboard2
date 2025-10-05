// Carte Leaflet
var map = L.map('map').setView([48.8566,2.3522], 15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);
var marker=L.marker([48.8566,2.3522]).addTo(map);
var polyline=L.polyline([], {color:'blue'}).addTo(map);

// Variables
let running=true;
let startTime=Date.now();
let prevLat=48.8566, prevLon=2.3522;
let totalDistance=0, speedMax=0;
let lastGPS=Date.now();
let lastAccel={x:0,y:0};
let velocity={x:0,y:0};
let uwbPos=null;
let tempData=[], speedData=[];

// Boutons
document.getElementById('toggleBtn').addEventListener('click', ()=>{running=!running; document.getElementById('toggleBtn').textContent=running?'Arrêt':'Marche';});
document.getElementById('resetBtn').addEventListener('click', ()=>{totalDistance=0; speedMax=0; prevLat=48.8566; prevLon=2.3522; velocity={x:0,y:0}; document.getElementById('speed').textContent='0'; document.getElementById('speedMax').textContent='0'; document.getElementById('distanceKm').textContent='0'; polyline.setLatLngs([]); tempData=[]; speedData=[]; chart.update();});
document.getElementById('bleBtn').addEventListener('click', connectBLE);

// Dead Reckoning
window.addEventListener('devicemotion', e=>{lastAccel.x=e.accelerationIncludingGravity.x||0; lastAccel.y=e.accelerationIncludingGravity.y||0;});

// Update position avec correction UWB
function updatePosition(deltaTime){
    velocity.x += lastAccel.x*deltaTime;
    velocity.y += lastAccel.y*deltaTime;

    let dLat=(velocity.y*deltaTime)/111000;
    let dLon=(velocity.x*deltaTime)/(111000*Math.cos(prevLat*Math.PI/180));

    prevLat+=dLat; prevLon+=dLon;
    totalDistance+=Math.sqrt((dLat*111000)**2 + (dLon*111000*Math.cos(prevLat*Math.PI/180))**2);

    if(uwbPos){
        const alpha=0.3;
        prevLat=prevLat*(1-alpha)+uwbPos.lat*alpha;
        prevLon=prevLon*(1-alpha)+uwbPos.lon*alpha;
    }

    marker.setLatLng([prevLat, prevLon]);
    polyline.addLatLng([prevLat, prevLon]);
    document.getElementById('lat').textContent=prevLat.toFixed(6);
    document.getElementById('lon').textContent=prevLon.toFixed(6);
    document.getElementById('distanceKm').textContent=(totalDistance/1000).toFixed(3);
    let speed=Math.sqrt(velocity.x**2+velocity.y**2);
    speedMax=Math.max(speedMax,speed);
    document.getElementById('speed').textContent=(speed*3.6).toFixed(1);
    document.getElementById('speedMax').textContent=(speedMax*3.6).toFixed(1);
    speedData.push(speed*3.6);
}

// GPS réel navigateur
function updateGPS(){
    if(navigator.geolocation){
        navigator.geolocation.getCurrentPosition(pos=>{
            prevLat=pos.coords.latitude; prevLon=pos.coords.longitude;
            lastGPS=Date.now();
            document.getElementById('lat').textContent=prevLat.toFixed(6);
            document.getElementById('lon').textContent=prevLon.toFixed(6);
            document.getElementById('alt').textContent=(pos.coords.altitude||0).toFixed(1);
            document.getElementById('gpsAcc').textContent=pos.coords.accuracy.toFixed(1);
            marker.setLatLng([prevLat,prevLon]);
            polyline.addLatLng([prevLat,prevLon]);
        });
    }
}

// BLE/UWB
async function connectBLE(){
    try{
        const device=await navigator.bluetooth.requestDevice({acceptAllDevices:true, optionalServices:['battery_service']});
        const server=await device.gatt.connect();
        const service=await server.getPrimaryService('battery_service');
        const char=await service.getCharacteristic('battery_level');
        char.startNotifications().then(c=>{
            c.addEventListener('characteristicvaluechanged', e=>{
                const val=e.target.value.getUint8(0);
                uwbPos={lat:48.8566+val*0.00001, lon:2.3522+val*0.00001};
                document.getElementById('mx').textContent=val;
            });
        });
    }catch(e){console.log(e); document.getElementById('headingDest').textContent='BLE erreur';}
}

// Horloge Minecraft
function mcClock(){let now=new Date(); let ticks=Math.floor(now.getTime()/50); let h=Math.floor(ticks/1000)%24; let m=Math.floor((ticks%1000)/1000*60); let s=Math.floor(((ticks%1000)/1000*60)%60); document.getElementById('mcClock').textContent=`${h.toString().padStart(2,'0')}:${m.toString().padStart(2,'0')}:${s.toString().padStart(2,'0')}`;}

// Canvas sismogramme et attitude
var seisCanvas=document.getElementById('seis'); var seisCtx=seisCanvas.getContext('2d');
function drawSismograph(){seisCtx.clearRect(0,0,seisCanvas.width,seisCanvas.height); seisCtx.beginPath(); for(let i=0;i<seisCanvas.width;i+=2){let y=seisCanvas.height/2+Math.sin(Date.now()/500+i/10)*30; if(i===0) seisCtx.moveTo(i,y); else seisCtx.lineTo(i,y);} seisCtx.strokeStyle='red'; seisCtx.stroke();}
var attCanvas=document.getElementById('attitude'); var attCtx=attCanvas.getContext('2d');
function drawAttitude(){attCtx.clearRect(0,0,attCanvas.width,attCanvas.height); let pitch=parseFloat(document.getElementById('ax').textContent)||0; let roll=parseFloat(document.getElementById('ay').textContent)||0; attCtx.fillStyle='blue'; attCtx.fillRect(attCanvas.width/2+roll*5, attCanvas.height/2+pitch*5,50,5);}

// Chart.js
var ctx=document.getElementById('tempChart').getContext('2d');
var chart=new Chart(ctx,{type:'line',data:{labels:[],datasets:[{label:'Temp °C',data:tempData,borderColor:'red',fill:false},{label:'Vitesse km/h',data:speedData,borderColor:'blue',fill:false}]},options:{animation:false,scales:{x:{display:false}}}});

// Mise à jour capteurs
let lastUpdate=Date.now();
function updateAll(){
    if(!running) return;
    let now=Date.now();
    let deltaTime=(now-lastUpdate)/1000;
    lastUpdate=now;

    mcClock(); drawSismograph(); drawAttitude();
    updateGPS();
    if(Date.now()-lastGPS>1000) updatePosition(deltaTime);

    // IMU, lumière, son (AmbientLightSensor et microphone)
    if('AmbientLightSensor' in window){let sensor=new AmbientLightSensor(); sensor.addEventListener('reading',()=>document.getElementById('lux').textContent=sensor.illuminance.toFixed(0)); sensor.start();}
    if(navigator.mediaDevices){navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{let context=new AudioContext(); let source=context.createMediaStreamSource(stream); let analyser=context.createAnalyser(); source.connect(analyser); analyser.fftSize=256; let dataArray=new Uint8Array(analyser.frequencyBinCount); function updateSound(){analyser.getByteFrequencyData(dataArray); let avg=dataArray.reduce((a,b)=>a+b)/dataArray.length; document.getElementById('dB').textContent=avg.toFixed(1); requestAnimationFrame(updateSound);} updateSound();}).catch(()=>{document.getElementById('dB').textContent='—';});}

    // Météo API
    let latC=prevLat??48.8566, lonC=prevLon??2.3522;
    fetch(`https://api.openweathermap.org/data/2.5/weather?lat=${latC}&lon=${lonC}&units=metric&appid=YOUR_API_KEY`).then(res=>res.json()).then(data=>{
        document.getElementById('temp').textContent=data.main.temp.toFixed(1);
        document.getElementById('pressure').textContent=data.main.pressure;
        document.getElementById('hum').textContent=data.main.humidity;
        document.getElementById('wind').textContent=data.wind.speed;
        tempData.push(data.main.temp);
        if(tempData.length>50){tempData.shift(); speedData.shift();}
        chart.update();
    }).catch(()=>{});

}

setInterval(updateAll,1000);
               
