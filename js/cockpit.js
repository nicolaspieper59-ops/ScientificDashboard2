window.addEventListener('load', () => {

    // Carte Leaflet
    var map = L.map('map').setView([48.8566, 2.3522], 15);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom:19}).addTo(map);
    var marker = L.marker([48.8566, 2.3522]).addTo(map);
    var polyline = L.polyline([], {color:'blue'}).addTo(map);

    // Variables
    let running = true;
    let prevLat = 48.8566, prevLon = 2.3522;
    let totalDistance = 0, speedMax = 0;
    let speed = 0;

    // Boutons
    const toggleBtn = document.getElementById('toggleBtn');
    toggleBtn.addEventListener('click', () => {
        running = !running;
        toggleBtn.textContent = running ? 'Marche' : 'Arrêt';
    });

    document.getElementById('resetBtn').addEventListener('click', () => {
        totalDistance = 0; speedMax = 0; speed = 0;
        prevLat = 48.8566; prevLon = 2.3522;
        document.getElementById('lat').textContent = prevLat.toFixed(6);
        document.getElementById('lon').textContent = prevLon.toFixed(6);
        document.getElementById('speed').textContent = '0';
        document.getElementById('speedMax').textContent = '0';
        document.getElementById('distanceKm').textContent = '0';
        marker.setLatLng([prevLat, prevLon]);
        polyline.setLatLngs([]);
    });

    // Horloge Minecraft
    function mcClock(){
        let now = new Date();
        let h = now.getHours().toString().padStart(2,'0');
        let m = now.getMinutes().toString().padStart(2,'0');
        let s = now.getSeconds().toString().padStart(2,'0');
        document.getElementById('mcClock').textContent = `${h}:${m}:${s}`;
    }

    // Graphique Chart.js
    var ctx = document.getElementById('tempChart').getContext('2d');
    var tempData = Array(50).fill(0);
    var speedData = Array(50).fill(0);
    var chart = new Chart(ctx,{
        type:'line',
        data:{
            labels: Array(50).fill(''),
            datasets:[
                {label:'Temp °C', data: tempData, borderColor:'red', fill:false},
                {label:'Vitesse km/h', data: speedData, borderColor:'blue', fill:false}
            ]
        },
        options:{animation:false, scales:{x:{display:false}}}
    });

    // Simulation GPS + vitesse
    function updateAll(){
        if(!running) return;

        // Simuler déplacement GPS
        prevLat += (Math.random()-0.5)/1000;
        prevLon += (Math.random()-0.5)/1000;
        speed = Math.random()*10;
        speedMax = Math.max(speedMax, speed);
        totalDistance += speed*0.0002778; // approx km/sec

        marker.setLatLng([prevLat, prevLon]);
        polyline.addLatLng([prevLat, prevLon]);

        document.getElementById('lat').textContent = prevLat.toFixed(6);
        document.getElementById('lon').textContent = prevLon.toFixed(6);
        document.getElementById('speed').textContent = speed.toFixed(1);
        document.getElementById('speedMax').textContent = speedMax.toFixed(1);
        document.getElementById('distanceKm').textContent = totalDistance.toFixed(3);

        // Simuler météo
        let temp = 20 + Math.random()*5;
        document.getElementById('temp').textContent = temp.toFixed(1);
        document.getElementById('pressure').textContent = (1010 + Math.random()*10).toFixed(0);
        document.getElementById('hum').textContent = (50 + Math.random()*10).toFixed(0);
        document.getElementById('wind').textContent = (5 + Math.random()*5).toFixed(1);

        tempData.push(temp); if(tempData.length>50) tempData.shift();
        speedData.push(speed); if(speedData.length>50) speedData.shift();
        chart.update();

        mcClock();
    }

    setInterval(updateAll,1000);
});
