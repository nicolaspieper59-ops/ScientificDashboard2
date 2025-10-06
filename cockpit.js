// ===========================
// VARIABLES GLOBALES
// ===========================
let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesseMax = 0;
let tempsDebut = null;

let accel = { z: 0 };
let altitudeBaro = null;

let capteursActifs = {
  gps: true,
  accel: true,
  baro: true
};

// ===========================
// UTILS
// ===========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

function toRad(deg) { return deg * Math.PI / 180; }

function haversine3D(p1, p2) {
  const R = 6371e3;
  const φ1 = toRad(p1.latitude), φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude - p1.latitude);
  const Δλ = toRad(p2.longitude - p1.longitude);
  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  const d2D = R * c;
  const dz = (p2.altitude ?? 0) - (p1.altitude ?? 0);
  return Math.sqrt(d2D**2 + dz**2);
}

// ===========================
// GPS
// ===========================
function miseAJourGPS(pos) {
  if (!capteursActifs.gps) return;

  const coords = pos.coords;
  const gps = {
    latitude: coords.latitude,
    longitude: coords.longitude,
    altitude: coords.altitude ?? (altitudeBaro ?? 0),
    accuracy: coords.accuracy,
    timestamp: pos.timestamp
  };

  safeSetText("gps", `Lat: ${gps.latitude.toFixed(6)} | Lon: ${gps.longitude.toFixed(6)} | Alt: ${gps.altitude.toFixed(2)} m | Précision: ${gps.accuracy.toFixed(1)} m`);

  if (!positionPrecedente) {
    positionPrecedente = gps;
    return;
  }

  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
  if (dt <= 0) return;

  const d = haversine3D(gps, positionPrecedente);
  distanceTotale += d;
  const v_ms = d / dt;
  const v_kmh = v_ms * 3.6;

  vitesseMax = Math.max(vitesseMax, v_kmh);
  const tempsEcoule = (Date.now() - tempsDebut) / 1000;

  safeSetText("vitesse", `Vitesse : ${v_kmh.toFixed(2)} km/h`);
  safeSetText("vitesse-max", `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText("distance", `Distance : ${(distanceTotale/1000).toFixed(3)} km`);
  safeSetText("temps", `Temps : ${tempsEcoule.toFixed(2)} s`);

  positionPrecedente = gps;
}

// ===========================
// CAPTEURS PHYSIQUES
// ===========================
function activerAccelerometre() {
  if (!capteursActifs.accel) return;

  if ("DeviceMotionEvent" in window) {
    window.addEventListener("devicemotion", e => {
      accel.z = e.accelerationIncludingGravity?.z ?? 0;
      safeSetText("accel", `Accélération Z : ${accel.z.toFixed(2)} m/s²`);
    });
  } else {
    safeSetText("accel", "Accéléromètre : ❌ Non supporté");
  }
}

function activerBarometre() {
  if (!capteursActifs.baro) return;

  if ("Barometer" in window) {
    try {
      const baro = new Barometer({ frequency: 1 });
      baro.addEventListener("reading", () => {
        const pression = baro.pressure;
        altitudeBaro = 44330 * (1 - Math.pow(pression / 1013.25, 1/5.255));
        safeSetText("baro", `Baromètre : ${pression.toFixed(2)} hPa (${altitudeBaro.toFixed(2)} m)`);
      });
      baro.start();
    } catch (err) {
      safeSetText("baro", "Baromètre : ❌ Non autorisé ou indisponible");
    }
  } else {
    safeSetText("baro", "Baromètre : ❌ Non supporté");
  }
}

// ===========================
// CONTRÔLE
// ===========================
function demarrer() {
  if (window.location.protocol !== "https:") {
    alert("⚠️ HTTPS requis pour activer le GPS et les capteurs !");
    return;
  }

  capteursActifs.gps = document.getElementById("useGPS").checked;
  capteursActifs.accel = document.getElementById("useAccel").checked;
  capteursActifs.baro = document.getElementById("useBaro").checked;

  tempsDebut = Date.now();
  distanceTotale = 0;
  vitesseMax = 0;
  positionPrecedente = null;

  if (capteursActifs.gps && "geolocation" in navigator) {
    watchId = navigator.geolocation.watchPosition(
      miseAJourGPS,
      err => safeSetText("gps", `Erreur GPS : ${err.message}`),
      { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
    );
  }

  activerAccelerometre();
  activerBarometre();
}

function arreter() {
  if (watchId) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  safeSetText("vitesse", "Vitesse : 0.00 km/h");
  safeSetText("gps", "GPS : --");
}

function reset() {
  distanceTotale = 0;
  vitesseMax = 0;
  safeSetText("distance", "Distance : 0.000 km");
  safeSetText("vitesse-max", "Vitesse max : 0.00 km/h");
}

// ===========================
// INIT
// ===========================
document.addEventListener("DOMContentLoaded", () => {
  document.getElementById("start").addEventListener("click", demarrer);
  document.getElementById("stop").addEventListener("click", arreter);
  document.getElementById("reset").addEventListener("click", reset);
});
    
