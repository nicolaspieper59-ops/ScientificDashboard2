// utils.js - petites fonctions utilitaires

function safeSetText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

// conversion m/s -> km/h
function msToKmh(v) { return v * 3.6; }

// Haversine 3D simple (m)
const R_EARTH = 6371e3;
function toRad(d) { return d * Math.PI / 180; }
function haversine3D(p1, p2) {
  const φ1 = toRad(p1.latitude), φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude - p1.latitude);
  const Δλ = toRad(p2.longitude - p1.longitude);
  const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const d2D = R_EARTH * c;
  const dz = (p2.altitude ?? 0) - (p1.altitude ?? 0);
  return Math.sqrt(d2D * d2D + dz * dz);
}
