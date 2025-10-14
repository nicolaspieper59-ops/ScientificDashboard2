// ========================
// 1. État global & Constantes
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;
let modeSouterrainActif = false;
let map = null;
let marker = null;
let targetCoords = { latitude: 48.8584, longitude: 2.2945 };

// Constantes physiques
const VITESSE_LUMIERE = 299792458;
const VITESSE_SON = 343;
const R_TERRE = 6371e3; 
const ANNEE_LUMIERE_SECONDES = 3600 * 24 * 365.25;
const SUN_RISE_SET_ALTITUDE = -0.833; // Altitude pour Lever/Coucher Soleil
const JD2000 = 2451545.0; // Jour Julien de J2000.0

// CLÉ API MÉTÉO (REMPLACEZ CETTE VALEUR)
const METEO_API_KEY = "VOTRE_CLE_API_ICI"; 
const METEO_API_URL = "https://api.openweathermap.org/data/2.5/weather?"; 

// ========================
// 2. Utilitaires & Géométrie
// ========================
function safeSetText(id, text) { 
    const e = document.getElementById(id); 
    const target = e?.querySelector('span') || e;
    if(target) target.textContent = text;
}
function toRadians(d){return d * Math.PI / 180;}
function toDegrees(r){return r * 180 / Math.PI;}

function calculerDistanceHaversine(p1,p2){
  const φ1=toRadians(p1.latitude), φ2=toRadians(p2.latitude);
  const Δφ=toRadians(p2.latitude-p1.latitude), Δλ=toRadians(p2.longitude-p1.longitude);
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return R_TERRE*2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
}

function calculerRelèvement(p1, p2) {
    const λ1 = toRadians(p1.longitude);
    const λ2 = toRadians(p2.longitude);
    const φ1 = toRadians(p1.latitude);
    const φ2 = toRadians(p2.latitude);
    const y = Math.sin(λ2 - λ1) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(λ2 - λ1);
    let brng = Math.atan2(y, x);
    brng = toDegrees(brng);
    return (brng + 360) % 360;
}

function setTargetCoords() {
    const input = document.getElementById('target-coord').value;
    const parts = input.split(',').map(p => parseFloat(p.trim()));
    if (parts.length === 2 && !isNaN(parts[0]) && !isNaN(parts[1])) {
        targetCoords = { latitude: parts[0], longitude: parts[1] };
        alert(`Cible définie : ${targetCoords.latitude.toFixed(4)}, ${targetCoords.longitude.toFixed(4)}`);
        if (positionPrecedente && watchId !== null) {
            calculerCible(positionPrecedente);
        }
    } else {
        alert("Format de coordonnées invalide. Utilisez 'lat, lon'.");
    }
}

// ========================
// 3. Pression et Température Réelles (API MÉTÉO)
// ========================

/**
 * Télécharge la pression (P_SL) et la température (T_local) via l'API Météo.
 */
async function telechargerPressionMeteo(latitude, longitude) {
    if (!METEO_API_KEY || METEO_API_KEY === "VOTRE_CLE_API_ICI") {
        return { pressionSL: 1013.25, temperatureC: 15, error: "Clé API manquante" };
    }
    const url = `${METEO_API_URL}lat=${latitude}&lon=${longitude}&appid=${METEO_API_KEY}&units=metric`;

    try {
        const response = await fetch(url);
        if (!response.ok) {
            throw new Error(`Erreur HTTP: ${response.status}`);
        }
        const data = await response.json();
        
        // Pression SL en hPa (hectopascals) et Température locale en Celsius
        return { 
            pressionSL: data.main.pressure, 
            temperatureC: data.main.temp,
            error: null
        };

    } catch (error) {
        console.error("Échec du téléchargement des données météo:", error);
        return { pressionSL: 1013.25, temperatureC: 15, error: "Échec API" };
    }
}

/**
 * Déduit la pression locale (Pression Station) à partir de la Pression SL, de l'Altitude GPS et de la Température API.
 */
function deduirePressionLocale(p_sl, altitude, t_local) {
    if (altitude <= 0) {
        return p_sl;
    }
    
    const T_Kelvin = t_local + 273.15; 
    const exposant = 5.257; 
    
    // Formule barométrique ajustée (simplifiée ISA)
    const p_local = p_sl * Math.pow(1 - (0.0065 * altitude) / T_Kelvin, exposant);
    
    return p_local;
}


// ========================
// 4. Calculs Astronomiques Réels
// ========================

function dateToJD(date) { 
    return (date.getTime() / 86400000.0) + 2440587.5;
}

function calculerHeuresSolaires(date, latitude, longitude) {
    const jd = dateToJD(date);
    const n = jd - JD2000;
    const J = n / 36525.0;

    const L_deg = (280.46646 + 36000.76983 * J + 0.0003032 * J * J) % 360;
    const M_deg = (357.52911 + 35999.05034 * J - 0.0001559 * J * J - 0.00000048 * J * J * J) % 360;
    const C = (1.914602 - 0.004817 * J - 0.000014 * J * J) * Math.sin(toRadians(M_deg)) +
              (0.019993 - 0.000101 * J) * Math.sin(toRadians(2 * M_deg)) +
              0.000289 * Math.sin(toRadians(3 * M_deg));
    const λ_s = L_deg + C;
    const ϵ = toRadians(23.439291 - 0.0130042 * J - 0.00000016 * J * J + 0.000000504 * J * J * J);

    const RA = toDegrees(Math.atan2(Math.cos(ϵ) * Math.sin(toRadians(λ_s)), Math.cos(toRadians(λ_s))));
    const RA_hours = (RA % 360) / 15;
    
    const EoT_hours = (L_deg % 360) / 15 - RA_hours;
    const EoT_seconds = EoT_hours * 3600;
    
    const heureUTC = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const HSM = (heureUTC + longitude / 15);
    const HSV = (HSM + EoT_hours) % 24;

    // Lever/Coucher du Soleil
    const cosH_num = Math.sin(toRadians(SUN_RISE_SET_ALTITUDE)) - Math.sin(toRadians(latitude)) * Math.sin(ϵ);
    const cosH_den = Math.cos(toRadians(latitude)) * Math.cos(ϵ);
    // Vérifie si le soleil se lève/couche (pas de soleil de minuit)
    if (Math.abs(cosH_num) < Math.abs(cosH_den)) {
        const H_deg = toDegrees(Math.acos(cosH_num / cosH_den));
        const T_rise_HSM = (12 - H_deg / 15) - EoT_hours;
        const T_set_HSM = (12 + H_deg / 15) - EoT_hours;
        const T_rise_locale = (T_rise_HSM - longitude / 15) % 24;
        const T_set_locale = (T_set_HSM - longitude / 15) % 24;
        safeSetText('lever-soleil', formatHeure(T_rise_locale));
        safeSetText('coucher-soleil', formatHeure(T_set_locale));
        safeSetText('solar-day-duration', formatHeure((T_set_locale - T_rise_locale) % 24)); 
    } else {
        safeSetText('lever-soleil', latitude > 0 ? "Jamais" : "Toujours");
        safeSetText('coucher-soleil', latitude > 0 ? "Toujours" : "Jamais");
        safeSetText('solar-day-duration', '24h 00min 00s');
    }

    // Affichage des données astronomiques
    safeSetText('date-display', date.toLocaleDateString('fr-FR', { weekday: 'long', year: 'numeric', month: 'long', day: 'numeric' }));
    safeSetText('hsv', formatHeure(HSV));
    safeSetText('hsm', formatHeure(HSM));
    safeSetText('eq-temps', `${(EoT_seconds).toFixed(2)} s`);
    safeSetText('solar-lon', `${λ_s.toFixed(2)}°`);
    
    // Placeholder Lunaire (Calculs complexes non inclus)
    safeSetText('phase-lune', `Non calculé`);
    safeSetText('culmination-lune', `Non calculé`);
    safeSetText('mag-lune', `Non calculé`);
}

function formatHeure(heureFloat) {
    const totalSecs = Math.round(heureFloat * 3600);
    const h = Math.floor(totalSecs / 3600);
    const m = Math.floor((totalSecs % 3600) / 60);
    const s = Math.floor(totalSecs % 60);
    return `${String(h).padStart(2, '0')}h ${String(m).padStart(2, '0')}min ${String(s).padStart(2, '0')}s`;
}


// ========================
// 5. Fonctions GPS & Orchestration
// ========================
async function miseAJourVitesse(pos) { // Déclaré async pour 'await'
    const now = Date.now();
    if (!tempsDebut) tempsDebut = now;

    // Filtre de base de précision (si la précision est trop mauvaise, ignorer)
    if (pos.coords.accuracy > 50 && !modeSouterrainActif) {
        console.warn(`Position ignorée: Précision de ${pos.coords.accuracy.toFixed(0)}m.`);
        return;
    }

    const gps = {
        latitude: pos.coords.latitude,
        longitude: pos.coords.longitude,
        // Utiliser l'altitude GPS pour la correction barométrique
        altitude: pos.coords.altitude || 0, 
        accuracy: pos.coords.accuracy,
        speed: pos.coords.speed || 0,
        heading: pos.coords.heading || 0
    };

    let vitesse_ms = gps.speed; 
    if (positionPrecedente) {
        // Calcul de distance cumulée si la position est valide
        distanceTotale += calculerDistanceHaversine(positionPrecedente, gps);
    } else {
        initialiserCarte(gps);
    }
    positionPrecedente = gps;

    const vitesse_kmh = vitesse_ms * 3.6;
    vitesseMax = Math.max(vitesseMax, vitesse_kmh);
    vitesses.push(vitesse_kmh); if (vitesses.length > 30) vitesses.shift();


    // ********* DÉMARCHE NON-SIMULÉE *********
    // 1. Récupération de la Pression et de la Température (API)
    const meteoData = await telechargerPressionMeteo(gps.latitude, gps.longitude);
    
    // 2. Déduction de la Pression Locale (Corrigée par Altitude et Température)
    const pressionLocaleEstimee = deduirePressionLocale(
        meteoData.pressionSL, 
        gps.altitude, 
        meteoData.temperatureC
    );

    // 3. Mise à jour des Capteurs (Réels ou API)
    afficherCapteurs(vitesse_kmh, pressionLocaleEstimee, meteoData.pressionSL, meteoData.temperatureC);
    
    // 4. Mise à jour Cinématique et Astro
    afficherVitesse(vitesse_kmh, vitesse_ms);
    afficherDistance();
    afficherTemps();
    afficherGPS(gps);
    calculerHeuresSolaires(new Date(), gps.latitude, gps.longitude);
    
    // 5. Carte et Cible
    mettreAJourCarte(gps); 
    calculerCible(gps);
}

function afficherVitesse(v_kmh, v_ms) {
    const moyenne_kmh = vitesses.reduce((a, b) => a + b, 0) / vitesses.length || 0;
    safeSetText('vitesse', `${v_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-moy', `${moyenne_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-max', `${vitesseMax.toFixed(2)} km/h`);
    safeSetText('vitesse-ms', `${v_ms.toFixed(2)} m/s | ${(v_ms * 1000).toFixed(0)} mm/s`);
    safeSetText('pourcentage', `${(v_ms / VITESSE_LUMIERE * 100).toExponential(2)}% | ${(v_ms / VITESSE_SON * 100).toFixed(2)}%`);
}

function afficherDistance() {
    const km = distanceTotale / 1000, m = distanceTotale, mm = m * 1000;
    const secLumiere = m / VITESSE_LUMIERE, al = secLumiere / ANNEE_LUMIERE_SECONDES;
    safeSetText('distance', `${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm`);
    safeSetText('distance-cosmique', `${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
    if (!tempsDebut) return;
    const t = (Date.now() - tempsDebut) / 1000;
    safeSetText('temps', `${t.toFixed(2)} s`);
}

function afficherGPS(g) {
    safeSetText('gps', `Lat : ${g.latitude.toFixed(4)} | Lon : ${g.longitude.toFixed(4)} | Alt : ${g.altitude.toFixed(0)} m | Préc. : ${g.accuracy.toFixed(0)} m`);
    safeSetText('compass-display', `${g.heading ? g.heading.toFixed(1) : '--'}° (GPS)`); // Utilise le heading GPS
}

function calculerCible(pos) {
    const bearing = calculerRelèvement(pos, targetCoords);
    const distance_m = calculerDistanceHaversine(pos, targetCoords);
    const distance_km = distance_m / 1000;
    safeSetText('bearing-display', `${bearing.toFixed(1)}° | Distance : ${distance_km.toFixed(3)} km`);
}

// ========================
// 6. Carte & Affichage des Capteurs
// ========================
function initialiserCarte(pos) {
    if (map) return;
    map = L.map('map').setView([pos.latitude, pos.longitude], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '© OpenStreetMap' }).addTo(map);
    marker = L.marker([pos.latitude, pos.longitude]).addTo(map);
}

function mettreAJourCarte(pos) {
    if (!map) initialiserCarte(pos);
    if (marker) {
        marker.setLatLng([pos.latitude, pos.longitude]);
        map.setView([pos.latitude, pos.longitude], map.getZoom(), { animate: true, pan: { duration: 0.5 } });
    }
}

/**
 * Affiche les valeurs de capteurs (Réels via API ou Non Supportés)
 */
let batterie = 85; // Simple simulation pour la batterie
function afficherCapteurs(v_kmh, pressionLocale, pressionSL, temperatureC) {
    // Pression & Température (via API)
    safeSetText('pression-est', `SL: ${pressionSL.toFixed(2)} hPa | Local: ${pressionLocale.toFixed(2)} hPa`);
    safeSetText('cap-lumiere', `${temperatureC.toFixed(1)} °C (API)`); // Réutilise le champ lumière pour afficher la température

    // Énergie Cinétique Est. : E = 0.5 * m * v^2 (Masse corps + équipement ≈ 80 kg)
    const energie = 0.5 * 80 * (v_kmh / 3.6)**2;
    safeSetText('energie-cinetique', `${energie.toFixed(0)} J`);

    // Autres Capteurs (Impossible en PWA sans API native)
    safeSetText('cap-son', `Non supporté`);
    safeSetText('cap-niveau', `Non supporté`);
    safeSetText('cap-gyro', `Non supporté`);
    safeSetText('cap-mag', `Non supporté`);
    safeSetText('cap-reseau', `API Externe`);
    
    // Heure et Jour/Nuit
    const now = new Date();
    safeSetText('heure-atomique', `${now.toUTCString().split(' ')[4]} UTC`);
    safeSetText('jour-nuit', (now.getHours() > 6 && now.getHours() < 20) ? 'Jour ☀️' : 'Nuit 🌙');

    // Batterie (Simulation)
    batterie = Math.max(0, batterie - 0.0001);
    safeSetText('cap-batterie', `${batterie.toFixed(1)}%`);
}


// ========================
// 7. Contrôles & Initialisation
// ========================
function demarrerCockpit(){
  if(watchId!==null) return;
  if(!tempsDebut) tempsDebut=Date.now();
  const options={enableHighAccuracy:true,timeout:10000,maximumAge:0}; // Timeout augmenté
  
  // Utilisation de la fonction asynchrone pour watchPosition
  watchId=navigator.geolocation.watchPosition(
    (pos) => miseAJourVitesse(pos),
    (e)=>{console.error(e);safeSetText('gps',`Erreur GPS (${e.code}) : ${e.message}`)},
    options
  );

  document.getElementById('marche').textContent = '⏸️ Pause';
}

function arreterCockpit(){
  if(watchId!==null){navigator.geolocation.clearWatch(watchId); watchId=null;}
  document.getElementById('marche').textContent = '▶️ Marche';
  safeSetText('vitesse','PAUSE');
}

function resetCockpit(){
    arreterCockpit(); positionPrecedente=null; vitesseMax=0; vitesses=[]; distanceTotale=0; tempsDebut=null;
    
    const initialText = ' --';
    safeSetText('vitesse', initialText + ' km/h'); safeSetText('vitesse-moy', initialText + ' km/h'); 
    safeSetText('vitesse-max', initialText + ' km/h'); 
    safeSetText('vitesse-ms', initialText + ' m/s |' + initialText + ' mm/s'); 
    safeSetText('pourcentage', initialText + '% |' + initialText + '%');
    safeSetText('distance', initialText + ' km |' + initialText + ' m |' + initialText + ' mm');
    safeSetText('distance-cosmique', initialText + ' s lumière |' + initialText + ' al');
    safeSetText('temps', initialText + ' s');
    safeSetText('gps', 'En attente...');
    safeSetText('compass-display', initialText + '°');
    safeSetText('bearing-display', initialText + '° | Distance : ' + initialText + ' km');
    
    // Reset Astro/Capteurs
    safeSetText('eq-temps', initialText + ' s');
    safeSetText('solar-lon', initialText + '°');
    safeSetText('hsv', initialText);
    safeSetText('hsm', initialText);
    safeSetText('lever-soleil', initialText);
    safeSetText('coucher-soleil', initialText);
    safeSetText('solar-day-duration', initialText);
    safeSetText('pression-est', initialText + ' hPa');
    safeSetText('energie-cinetique', initialText + ' J');
    
    // Initialisation des valeurs non supportées
    afficherCapteurs(0, 1013.25, 1013.25, 15);
}

document.addEventListener('DOMContentLoaded', () => {
    document.getElementById('marche').addEventListener('click', demarrerCockpit);
    document.getElementById('arreter').addEventListener('click', arreterCockpit);
    document.getElementById('reset').addEventListener('click', resetCockpit);
    document.getElementById('toggle-souterrain').addEventListener('click', () => {
        modeSouterrainActif = !modeSouterrainActif;
        document.getElementById('toggle-souterrain').textContent = `🚫 Mode Souterrain : ${modeSouterrainActif ? 'ON' : 'OFF'}`;
        if (modeSouterrainActif) arreterCockpit();
    });

    document.getElementById('target-coord').value = `${targetCoords.latitude}, ${targetCoords.longitude}`;
    
    resetCockpit();
});
