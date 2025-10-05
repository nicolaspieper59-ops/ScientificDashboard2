window.addEventListener('load', () => {

    // Carte Leaflet
    var map = L.map('map').setView([48.8566, 2.3522], 15);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom:19}).addTo(map);
    var marker = L.marker([48.8566, 2.3522]).addTo(map);
    var polyline = L.polyline([], {color:'blue'}).addTo(map);

    // Variables globales
    let running = true, prevLat=48.8566, prevLon=2.3522, totalDistance=0, speedMax=0, velocity={x:0,y:0}, lastGPS=Date.now(), lastAccel={x:0,y:0}, uwbPos=null;
    const toggleBtn=document.getElementById('toggleBtn');
    toggleBtn.textContent='Marche';
    toggleBtn.addEventListener('click',()=>{running=!running;toggleBtn.textContent=running?'Marche':'Arrêt';});
    document.getElementById('resetBtn').addEventListener('click',()=>{totalDistance=0; speedMax=0; prevLat=48.8566; prevLon=2.3522; velocity={x:0,y:0}; polyline.setLatLngs([]);});

    // BLE/UWB basique
    document.getElementById('bleBtn').addEventListener('click', async()=>{
        try{
            const device = await navigator.bluetooth.requestDevice({acceptAllDevices:true});
            const server = await device.gatt.connect();
            // Simuler position UWB
            uwbPos={lat:48.8566, lon:2.3522};
        }catch(e){console.log('BLE erreur',e);}
    });

    // IMU
    window.addEventListener('devicemotion', e=>{lastAccel.x=e.accelerationIncludingGravity?.x||0; lastAccel.y=e.accelerationIncludingGravity?.y||0;});

    // Horloge Minecraft
    function mcClock(){let now=new Date();document.getElementById('mcClock').textContent=`${now.getHours().toString().padStart(2,'0')}:${now.getMinutes().toString().padStart(2,'0')}:${now.getSeconds().toString().padStart(2,'0')}`;}

    // Graphique Chart.js
    var ctx=document.getElementById('tempChart').getContext('2d'), tempData=[], speedData=[];
    var chart=new Chart(ctx,{type:'line',data:{labels:Array(50).fill(''),datasets:[{label:'Temp °C',data:tempData,borderColor:'red',fill:false},{label:'Vitesse km/h',data:speedData,borderColor:'blue',fill:false}]},options:{animation:false,scales:{x:{display:false}}}});

    // Sismogramme
    var seisCtx=document.getElementById('seis').getContext('2d');
    function drawSismograph(){seisCtx.clearRect(0,0,600,180);seisCtx.beginPath();for(let i=0;i<600;i+=2){let y=90+Math.sin(Date.now()/500+i/10)*30;if(i===0)seisCtx.moveTo(i,y);else seisCtx.lineTo(i,y);}seisCtx.strokeStyle='red';seisCtx.stroke();}

    // Attitude
    var attCtx=document.getElementById('attitude').getContext('2d');
    function drawAttitude(){attCtx.clearRect(0,0,600,180);attCtx.fillStyle='blue';attCtx.fillRect(300,90,50,5);}

    // GPS réel + Dead Reckoning
    function updatePosition(deltaTime){velocity.x+=lastAccel.x*deltaTime; velocity.y+=lastAccel.y*deltaTime; let dLat=(velocity.y*deltaTime)/111000; let dLon=(velocity.x*deltaTime)/(111000*Math.cos(prevLat*Math.PI/180)); prevLat+=dLat; prevLon+=dLon; totalDistance+=Math.sqrt((dLat*111000)**2+(dLon*111000*Math.cos(prevLat*Math.PI/180))**2); if(uwbPos){const a=0.3; prevLat=prevLat*(1-a)+uwbPos.lat*a; prevLon=prevLon*(1-a)+uwbPos.lon*a;} marker.setLatLng([prevLat,prevLon]); polyline.addLatLng([prevLat,prevLon]); let speed=Math.sqrt(velocity.x**2+velocity.y**2); speedMax=Math.max(speedMax,speed); document.getElementById('speed').textContent=(speed*3.6).toFixed(1); document.getElementById('speedMax').textContent=(speedMax*3.6).toFixed(1); document.getElementById('distanceKm').textContent=(totalDistance/1000).toFixed(3);}

    function updateGPS(){if(navigator.geolocation){navigator.geolocation.getCurrentPosition(pos=>{prevLat=pos.coords.latitude; prevLon=pos.coords.longitude; lastGPS=Date.now(); document.getElementById('lat').textContent=prevLat.toFixed(6); document.getElementById('lon').textContent=prevLon.toFixed(6); document.getElementById('alt').textContent=(pos.coords.altitude||0).toFixed(1); document.getElementById('gpsAcc').textContent=pos.coords.accuracy.toFixed(1); marker.setLatLng([prevLat,prevLon]); polyline.addLatLng([prevLat,prevLon]);});}}

    // Lumière
    if('AmbientLightSensor' in window){let sensor=new AmbientLightSensor();sensor.addEventListener('reading',()=>document.getElementById('lux').textContent=sensor.illuminance.toFixed(0));sensor.start();}

    // Son
    if(navigator.mediaDevices){navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{let context=new AudioContext();let source=context.createMediaStreamSource(stream);let analyser=context.createAnalyser();source.connect(analyser);analyser.fftSize=256;let dataArray=new Uint8Array(analyser.frequencyBinCount);function updateSound(){analyser.getByteFrequencyData(dataArray);let avg=dataArray.reduce((a,b)=>a+b)/dataArray.length;document.getElementById('dB').textContent=avg.toFixed(1);requestAnimationFrame(updateSound);}updateSound();}).catch(()=>{document.getElementById('dB').textContent='—';});}

    // Météo OpenWeatherMap
    const API_KEY='YOUR_API_KEY';
    function updateWeather(){let latC=prevLat||48.8566;let lonC=prevLon||2.3522;fetch(`https://api.openweathermap.org/data/2.5/weather?lat=${latC}&lon=${lonC}&units=metric&appid=${API_KEY}`).then(res=>res.json()).then(data=>{document.getElementById('temp').textContent=data.main.temp.toFixed(1);document.getElementById('pressure').textContent=data.main.pressure;document.getElementById('hum').textContent=data.main.humidity;document.getElementById('wind').textContent=data.wind.speed;tempData.push(data.main.temp);if(tempData.length>50)tempData.shift();speedData.push(parseFloat(document.getElementById('speed').textContent));if(speedData.length>50)speedData.shift();chart.update();}).catch(()=>{});}

    // Boucle principale
    let lastUpdate=Date.now();
    function updateAll(){
        if(!running) return;
        let now=Date.now(), deltaTime=(now-lastUpdate)/1000;
        lastUpdate=now;
        mcClock(); drawSismograph(); drawAttitude();
        updateGPS(); if(Date.now()-lastGPS>1000) updatePosition(deltaTime);
        updateWeather();
    }

    setInterval(updateAll,1000);
});
        
