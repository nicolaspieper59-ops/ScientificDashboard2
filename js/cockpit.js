// cockpitCosmique.js
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;

let destination = { latitude: null, longitude: null };

/* ========================
   UTILITAIRES DOM SÉCURISÉS
======================== */
function safeSetText(id, text) {
    const el = document.getElementById(id);
    if (el) el.textContent = text;
}

function appendText(id, text) {
    const el = document.getElementById(id);
    if (el) {
        el.textContent = (el.textContent.length > 1000) ? text : (el.textContent + text);
    }
}

const _baseTexts = {};
function setElementTextBase(id, base) {
    _baseTexts[id] = base;
    const el = document.getElementById(id);
    if (el) el.textContent = base;
}
function getElementText(id) {
    const el = document.getElementById(id);
    return el ? el.textContent : (_baseTexts[id] || '');
}

/* ========================
   GPS / VITESSE / DISTANCE
======================== */
export function demarrerCockpit() {
    if (!("geolocation" in navigator)) {
        safeSetText('gps', 'GPS non disponible');
        return;
    }
    if (watchId !== null) return;

    watchId = navigator.geolocation.watchPosition(pos => {
        const gps = {
            latitude: pos.coords.latitude,
            longitude: pos.coords.longitude,
            altitude: pos.coords.altitude,
            accuracy: pos.coords.accuracy,
            timestamp: pos.timestamp,
            speed: pos.coords.speed
        };

        const vitesse = typeof gps.speed === 'number' && gps.speed !== null
            ? gps.speed * 3.6
            : calculerVitesse(gps);

        if (vitesse >= 0 && vitesse < 300) {
            vitesseMax = Math.max(vitesseMax, vitesse);
            vitesses.push(vitesse);

            afficherVitesse(vitesse);
            afficherDistance();
            afficherPourcentage(vitesse);
            afficherGPS(gps);
        }
    }, err => {
        console.error('Erreur GPS :', err);
        safeSetText('gps', 'Erreur GPS : ' + (err.message || err.code || 'inconnue'));
    }, { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 });
}

export function arreterCockpit() {
    if (watchId !== null && 'geolocation' in navigator) {
        navigator.geolocation.clearWatch(watchId);
    }
    watchId = null;
    positionPrecedente = null;
    vitesses = [];
    distanceTotale = 0;
    vitesseMax = 0;
}

export function resetVitesseMax() {
    vitesseMax = 0;
}

/* ========================
   CALCULS
======================== */
function calculerVitesse(gps) {
    if (!positionPrecedente) {
        positionPrecedente = gps;
        return 0;
    }

    const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000; // s
    const d = calculerDistance(gps, positionPrecedente); // m
    if (!Number.isFinite(d) || d < 0) {
        positionPrecedente = gps;
        return 0;
    }
    distanceTotale += d;
    positionPrecedente = gps;

    return dt > 0 ? (d / dt) * 3.6 : 0;
}

function calculerDistance(pos1, pos2) {
    const R = 6371e3;
    const φ1 = pos1.latitude * Math.PI / 180;
    const φ2 = pos2.latitude * Math.PI / 180;
    const Δφ = (pos2.latitude - pos1.latitude) * Math.PI / 180;
    const Δλ = (pos2.longitude - pos1.longitude) * Math.PI / 180;

    const a = Math.sin(Δφ / 2) ** 2 +
        Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

/* ========================
   AFFICHAGE
======================== */
function afficherVitesse(vitesse) {
    const moyenne = vitesses.length ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length : 0;
    const mps = vitesse / 3.6;
    const mmps = mps * 1000;

    safeSetText('vitesse',
        `Temps : ${new Date().toLocaleTimeString()} | ` +
        `Vitesse instantanée : ${vitesse.toFixed(2)} km/h | ` +
        `Moyenne : ${moyenne.toFixed(2)} km/h | ` +
        `Max : ${vitesseMax.toFixed(2)} km/h | ` +
        `${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`
    );
}

function afficherDistance() {
    const km = distanceTotale / 1000;
    const m = distanceTotale;
    const mm = m * 1000;
    const secondesLumiere = m / 299792458;
    const anneesLumiere = secondesLumiere / (3600 * 24 * 365.25);

    safeSetText('distance',
        `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ` +
        `${mm.toFixed(0)} mm | Cosmique : ${secondesLumiere.toFixed(3)} s lumière | ` +
        `${anneesLumiere.toExponential(3)} al`
    );
}

function afficherPourcentage(vitesse) {
    const mps = vitesse / 3.6;
    const pourcentLumiere = (mps / 299792458) * 100;
    const pourcentSon = (mps / 343) * 100;

    safeSetText('pourcentage',
        `% Lumière : ${pourcentLumiere.toExponential(2)}% | ` +
        `% Son : ${pourcentSon.toFixed(2)}%`
    );
}

function afficherGPS(gps) {
    safeSetText('gps',
        `Latitude : ${gps.latitude.toFixed(6)} | ` +
        `Longitude : ${gps.longitude.toFixed(6)} | ` +
        `Altitude : ${(gps.altitude ?? '--') === '--' ? '--' : (gps.altitude).toFixed(1) + ' m'} | ` +
        `Précision GPS : ${gps.accuracy != null ? gps.accuracy.toFixed(0) + '%' : '--'}`
    );
}

/* ========================
   CAPTEURS PHYSIQUES (après clic)
======================== */
function activerCapteurs() {
    setElementTextBase('capteurs', 'Niveau à bulle : --° | Lumière : -- lux');

    // Lumière
    if ('AmbientLightSensor' in window) {
        try {
            const sensor = new AmbientLightSensor();
            sensor.addEventListener('reading', () => {
                appendText('capteurs', ` | Lumière : ${Math.round(sensor.illuminance)} lux`);
            });
            sensor.start();
        } catch (err) { console.warn(err); }
    }

    // Microphone
    if (navigator.mediaDevices?.getUserMedia) {
        navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
            const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
            const analyser = audioCtx.createAnalyser();
            const source = audioCtx.createMediaStreamSource(stream);
            source.connect(analyser);
            analyser.fftSize = 2048;
            const data = new Uint8Array(analyser.frequencyBinCount);

            function analyserSon() {
                analyser.getByteFrequencyData(data);
                const moyenne = data.reduce((a,b)=>a+b,0)/data.length;
                const dB = 20*Math.log10(moyenne||1);
                safeSetText('capteurs', getElementText('capteurs') + ` | Son : ${Math.round(dB)} dB`);
                requestAnimationFrame(analyserSon);
            }
            analyserSon();
        }).catch(err => console.warn('Microphone indisponible', err));
    }

    // Orientation (niveau à bulle)
    if (typeof DeviceOrientationEvent !== 'undefined') {
        if (typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permission => {
                    if(permission === 'granted') {
                        window.addEventListener('deviceorientation', e => {
                            appendText('capteurs', ` | Niveau à bulle : ${e.beta?.toFixed(1)}°`);
                        });
                    }
                }).catch(console.warn);
        } else {
            window.addEventListener('deviceorientation', e => {
                appendText('capteurs', ` | Niveau à bulle : ${e.beta?.toFixed(1)}°`);
            });
        }
    }

    // Magnétomètre
    if ('Magnetometer' in window) {
        try {
            const magneto = new Magnetometer();
            magneto.addEventListener('reading', () => {
                const champ = Math.sqrt(magneto.x**2 + magneto.y**2 + magneto.z**2);
                appendText('capteurs', ` | Magnétisme : ${Math.round(champ)} µT`);
            });
            magneto.start();
        } catch(err){ console.warn(err); }
    }
}

/* ========================
   MÉDAILLON & HORLOGE
======================== */
function afficherMedaillon() {
    const medaillon = document.getElementById('medaillon');
    if(!medaillon) return;
    medaillon.innerHTML = '';
    const canvas = document.createElement('canvas');
    canvas.width = 300; canvas.height = 300;
    medaillon.appendChild(canvas);
    const ctx = canvas.getContext('2d');

    // Fond
    ctx.fillStyle = '#000'; ctx.fillRect(0,0,300,300);
    ctx.strokeStyle = '#444'; ctx.lineWidth=2;
    ctx.beginPath(); ctx.arc(150,150,140,0,2*Math.PI); ctx.stroke();

    // Soleil
    ctx.fillStyle='#ffd700'; ctx.beginPath();
    ctx.arc(210,90,8,0,2*Math.PI); ctx.fill();
    ctx.fillStyle='#fff'; ctx.font='10px monospace'; ctx.fillText('☀️ Soleil',200,80);

    // Lune
    ctx.fillStyle='#ccc'; ctx.beginPath();
    ctx.arc(90,110,6,0,2*Math.PI); ctx.fill();
    ctx.fillText('🌙 Lune',80,100);
}

function activerHorlogeMinecraft() {
    const horloge = document.getElementById('horloge');
    if(!horloge) return;
    function miseAJour() {
        const now = new Date();
        const h = String(now.getHours()).padStart(2,'0');
        const m = String(now.getMinutes()).padStart(2,'0');
        const s = String(now.getSeconds()).padStart(2,'0');
        horloge.textContent = `⏰ Horloge Minecraft\n${h}:${m}:${s}`;
        requestAnimationFrame(miseAJour);
    }
    miseAJour();
}

/* ========================
   INIT BOUTONS
======================== */
document.addEventListener('DOMContentLoaded', ()=>{
    const boutonMarche = document.getElementById('marche');
    const boutonReset = document.getElementById('reset');

    if(boutonMarche){
        boutonMarche.addEventListener('click', ()=>{
            demarrerCockpit();
            activerCapteurs();
            afficherMedaillon();
            activerHorlogeMinecraft();
        });
    }

    if(boutonReset){
        boutonReset.addEventListener('click', ()=>{
            resetVitesseMax();
            appendText('vitesse',' | Max réinitialisé');
        });
    }
});
        
