// Carte Leaflet
var map = L.map('map').setView([48.8566, 2.3522], 15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom:19}).addTo(map);
var marker = L.marker([48.8566,2.3522]).addTo(map);
var polyline = L.polyline([], {color:'blue'}).addTo(map);

// Variables
let running = true;
let prevLat = 48.8566, prevLon = 2.3522;
let totalDistance = 0, speedMax = 0;
let lastGPS = Date.now();
let lastAccel = {x:0,y:0};
let velocity = {x:0,y:0};
let uwbPos = null;
let tempData = [], speedData = [];

// Boutons
document.getElementById('toggleBtn').addEventListener('
                                                      
