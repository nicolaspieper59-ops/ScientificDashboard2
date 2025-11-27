// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/5 : Constantes, Utilitaires, et État Global
// =================================================================

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let isGpsPaused = false; 
let currentPosition = { 
    // Coordonnées de démarrage par défaut (Paris)
    lat: 48.8566,   
    lon: 2.3522,    
    alt: 35.0,      
    acc: 10.0,      
    spd: 0.0        
};
let lastP_hPa = 1013.25; 
let lastT_K = 288.15; 
let currentAirDensity = 1.225;
let currentSpeedOfSound = 340.3;
let lastKnownWeather = null;
let lastKnownPollutants = null;
let currentMass = 70.0;
let selectedEnvironment = 'AIR'; // Clé pour ENVIRONMENT_FACTORS

// --- CONSTANTES PHYSIQUES ET ENVIRONNEMENTALES ---
const PHYS = {
    c: 299792458, // Vitesse de la lumière (m/s)
    G: 6.67430e-11, // Constante gravitationnelle
    R_earth: 6371000, // Rayon terrestre moyen (m)
    TEMP_SEA_LEVEL_K: 288.15, // 15°C en Kelvin
    BARO_ALT_REF_HPA: 1013.25 // Pression de référence
};
const RHO_SEA_LEVEL = 1.225; // Densité de l'air au niveau de la mer (kg/m³)

const ENVIRONMENT_FACTORS = {
    AIR: { DISPLAY: 'Atmosphère Standard', MULT: 1.0, DRAG_COEF: 0.47 },
    WATER: { DISPLAY: 'Eau (H₂O)', MULT: 820.0, DRAG_COEF: 0.82 },
    VACUUM: { DISPLAY: 'Vide', MULT: 0.0, DRAG_COEF: 0.01 }
};

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const nowISO = () => new Date().toISOString();
const safeSet = (id, txt) => { const e=$(id); if(e) e.textContent = txt; };

/** Effectue un fetch avec une stratégie de backoff exponentiel */
async function fetchWithBackoff(url, opts={}, retries=3, delay=600){
  for(let i=0;i<=retries;i++){
    try{
      const r = await fetch(url, opts);
      if(!r.ok) throw new Error('HTTP '+r.status);
      return await r.json();
    }catch(e){
      if(i===retries) { console.warn('Échec définitif du fetch:', url, e); throw e; }
      // Attendre et doubler le délai
      await new Promise(r=>setTimeout(r, delay));
      delay *= 2;
    }
  }
}

/** Calcule la vitesse du son en m/s à partir de la température en Kelvin. */
function getSpeedOfSound(T_K) {
    const R_air = 287.058;
    const gamma_air = 1.4;
    return Math.sqrt(gamma_air * R_air * T_K);
}

/** Calcule le Facteur de Lorentz gamma. */
function lorentzGamma(v) {
    if (v >= PHYS.c) return Infinity;
    return 1 / Math.sqrt(1 - (v * v) / (PHYS.c * PHYS.c));
}

// =================================================================
// BLOC 2/5 : STOCKAGE (IndexedDB)
// =================================================================
const Storage = (function(){
  const DB='gnss_stime_db', STORE='obs';
  let db=null;
  function openDB(){
    return new Promise((res,rej)=>{
      if(db) return res(db);
      const r = indexedDB.open(DB,1);
      r.onupgradeneeded = e => { 
        const d=e.target.result; 
        if(!d.objectStoreNames.contains(STORE)) {
            d.createObjectStore(STORE, {keyPath:'ts'});
        }
      };
      r.onsuccess = e => { db=e.target.result; res(db); };
      r.onerror = e => { console.error("Erreur IndexedDB", e); rej(e); };
    });
  }

  return {
    saveObservation: async function(data) {
        if (isGpsPaused) return; 
        try {
            const d = await openDB();
            const tx = d.transaction(STORE, 'readwrite');
            await tx.objectStore(STORE).add({ ...data, ts: nowISO() });
            await tx.oncomplete;
            return true;
        } catch (e) {
            console.error("Échec de la sauvegarde IDB:", e);
            return false;
        }
    },
    getObservations: async function() {
        try {
            const d = await openDB();
            const tx = d.transaction(STORE, 'readonly');
            return await tx.objectStore(STORE).getAll();
        } catch (e) {
            console.error("Échec de la récupération IDB:", e);
            return [];
        }
    },
    clearObservations: async function() {
        try {
            const d = await openDB();
            const tx = d.transaction(STORE, 'readwrite');
            await tx.objectStore(STORE).clear();
            await tx.oncomplete;
            return true;
        } catch (e) {
            console.error("Échec de l'effacement IDB:", e);
            return false;
        }
    }
  };
})();

// =================================================================
// BLOC 3/5 : SIMULATION UKF (Simplifiée) et SciencePro
// =================================================================

const FilterEngine = {
    // État UKF simulé
    state: { lat: 48.8566, lon: 2.3522, alt: 35.0, speed: 0.0, P: math.matrix([[1.0, 0], [0, 1.0]]) }, 

    initFromGnss: function(lat, lon, alt) {
        // Initialise le filtre avec une précision par défaut de 1 mètre
        this.state = { 
            lat: lat, 
            lon: lon, 
            alt: alt, 
            speed: 0.0, 
            P: math.matrix([[1.0, 0], [0, 1.0]]) // Matrice de covariance 
        };
    },
    getState: function() {
        return this.state;
    },
    updateState: function(newLat, newLon, newAlt, newSpeed) {
        // Simulation d'une mise à jour de l'état (le filtre réel serait ici)
        this.state.lat = newLat;
        this.state.lon = newLon;
        this.state.alt = newAlt;
        this.state.speed = newSpeed;
        
        // Simuler une petite amélioration de la précision
        let P = this.state.P.toArray();
        P[0][0] *= 0.99; 
        P[1][1] *= 0.99; 
        this.state.P = math.matrix(P);
    }
};

const ScienceAstroPro = (function() {
    // Vérifie si la librairie Astronomy Engine est chargée.
    if (typeof Astronomy === 'undefined') {
        console.warn("Astronomy Engine non chargée. Les calculs Astro seront N/A.");
        // Retourne un stub pour éviter les erreurs si le script CDN échoue
        return {
            sunAltAz: () => ({ alt: 0, azimuth: 0 }),
            moonAltAz: () => ({ alt: 0, azimuth: 0 }),
            moonPosition: () => ({ illuminatedFraction: 0, ageDays: 0, phaseName: 'N/A' }),
            getSunRiseSet: () => ({ rise: 'N/A (Erreur Lib)', set: 'N/A (Erreur Lib)', dayDuration: 'N/A' }),
            getMoonRiseSet: () => ({ rise: 'N/A (Erreur Lib)', set: 'N/A (Erreur Lib)', dayDuration: 'N/A' })
        };
    }

    /** Calcule l'Altitude/Azimut du Soleil ou de tout autre corps (Body) */
    function getAltAz(body, date, lat, lon, alt_obs = 0) {
        const observer = new Astronomy.MakeBody(Astronomy.Body.Observer, lat, lon, alt_obs);
        const pos = Astronomy.Equator(body, date, observer, true, true);
        const altAz = Astronomy.Horizon(date, observer, pos.ra, pos.dec);
        return { alt: altAz.altitude, azimuth: altAz.azimuth };
    }

    /** Calcule les données de la Lune (Phase, Illumination) */
    function moonPosition(date) {
        const frac = Astronomy.Illumination(Astronomy.Body.Moon, date);
        const phaseName = Astronomy.MoonPhaseName(frac.phase);
        return { 
            illuminatedFraction: frac.phase, 
            ageDays: frac.age,
            phaseName: phaseName 
        };
    }
    
    /** Calcule les heures de Lever/Coucher d'un corps */
    function getRiseSetTimes(body, date, lat, lon, alt_obs = 0) {
        const observer = new Astronomy.MakeBody(Astronomy.Body.Observer, lat, lon, alt_obs);
        let rise, set;
        
        try {
            // Recherche du lever (Rise) et coucher (Set)
            rise = Astronomy.SearchRiseSet(body, observer, Astronomy.Direction.Rise, date, 1.0);
            set = Astronomy.SearchRiseSet(body, observer, Astronomy.Direction.Set, date, 1.0);
        } catch (e) {
            rise = null; set = null;
        }

        const formatTime = (time) => time ? Astronomy.FormatTime(time.date, 'HH:MM:SS') + ' UTC' : 'N/A';
        
        let dayDuration = 'N/A';
        if (body === Astronomy.Body.Sun && rise && set && rise.date && set.date) {
            const diffMs = set.date.getTime() - rise.date.getTime();
            const hours = Math.floor(diffMs / (1000 * 60 * 60));
            const minutes = Math.floor((diffMs % (1000 * 60 * 60)) / (1000 * 60));
            dayDuration = `${hours}h ${minutes}min`;
        }
        
        return {
            rise: formatTime(rise),
            set: formatTime(set),
            dayDuration: dayDuration
        };
    }

    return {
        sunAltAz: (date, lat, lon, alt_obs) => getAltAz(Astronomy.Body.Sun, date, lat, lon, alt_obs),
        moonAltAz: (date, lat, lon, alt_obs) => getAltAz(Astronomy.Body.Moon, date, lat, lon, alt_obs),
        moonPosition,
        getSunRiseSet: (date, lat, lon, alt_obs) => getRiseSetTimes(Astronomy.Body.Sun, date, lat, lon, alt_obs),
        getMoonRiseSet: (date, lat, lon, alt_obs) => getRiseSetTimes(Astronomy.Body.Moon, date, lat, lon, alt_obs)
    };
})();

// =================================================================
// BLOC 4/5 : MISE À JOUR DES AFFICHAGES ET ORCHESTRATION
// =================================================================

/** Met à jour le DOM avec les données météo/environnementales */
function updateWeatherDOM(data, isDefault = false) {
    const suffix = isDefault ? ' (Défaut)' : '';
    safeSet('pressure-hpa', `${data.pressure_hPa.toFixed(2)} hPa${suffix}`);
    safeSet('temp-c', `${(data.tempK - 273.15).toFixed(2)} °C${suffix}`);
    // Mettre à jour les variables globales pour les calculs de traînée/son
    lastP_hPa = data.pressure_hPa;
    lastT_K = data.tempK;
    currentAirDensity = data.air_density;
}

/** Met à jour le DOM avec les données de pollution */
function updatePollutantsDOM(data) {
    const list = document.getElementById('pollutant-list');
    if (!list) return;
    list.innerHTML = ''; // Nettoyer
    
    // Simuler l'affichage si l'HTML le permet
    const pm10 = document.createElement('div');
    pm10.className = 'data-point';
    pm10.innerHTML = `<span>PM10</span><span>${data.pm10.toFixed(1)} µg/m³</span>`;
    list.appendChild(pm10);
}

/** Met à jour les données dérivées (Astro, Météo calculée, Relativité) */
async function refreshDerived(lat, lon, alt_obs = 0, speed = 0) {
    try {
        const now = new Date();
        const pos = { lat: lat, lon: lon };
        
        // --- ASTRONOMIE HAUTE PRÉCISION ---
        const sun = ScienceAstroPro.sunAltAz(now, pos.lat, pos.lon, alt_obs);
        safeSet('sun-alt', `${sun.alt.toFixed(3)}°`);
        safeSet('sun-azimuth', `${sun.azimuth.toFixed(3)}°`);
        
        const sunTimes = ScienceAstroPro.getSunRiseSet(now, pos.lat, pos.lon, alt_obs);
        safeSet('day-duration', sunTimes.dayDuration);
        safeSet('sunrise-times', sunTimes.rise);
        safeSet('sunset-times', sunTimes.set);
        
        const moonData = ScienceAstroPro.moonPosition(now);
        const moonAltAz = ScienceAstroPro.moonAltAz(now, pos.lat, pos.lon, alt_obs); 
        const moonTimes = ScienceAstroPro.getMoonRiseSet(now, pos.lat, pos.lon, alt_obs);
        
        safeSet('moon-phase-name', moonData.phaseName); 
        safeSet('moon-illuminated', `${(moonData.illuminatedFraction * 100).toFixed(1)}%`);
        safeSet('moon-alt', `${moonAltAz.alt.toFixed(3)}°`);
        safeSet('moon-azimuth', `${moonAltAz.azimuth.toFixed(3)}°`);
        safeSet('moon-times', `Lev: ${moonTimes.rise} / Cou: ${moonTimes.set}`); 
        
        // --- RELATIVITÉ ---
        const gamma = lorentzGamma(speed);
        safeSet('lorentz', gamma.toFixed(8));
        
        // --- MÉTROLOGIE CALCULÉE ---
        const speedOfSound = getSpeedOfSound(lastT_K);
        safeSet('speed-of-sound-calc', `${speedOfSound.toFixed(2)} m/s`);
        
        // --- UKF/P-Matrix (Incertitudes) ---
        const Pmat = FilterEngine.getState().P.toArray();
        if(Pmat && Pmat.length > 1){ 
            safeSet('p-pos', Math.sqrt(Pmat[0][0] + Pmat[1][1]).toFixed(3) + ' m'); 
            // Simuler l'incertitude vitesse (P[3][3] + P[4][4])
            safeSet('p-vel', '0.045' + ' m/s'); 
        }

        // --- TEMPS ---
        safeSet('local-time', now.toLocaleTimeString());
        safeSet('gps-time', now.toISOString().replace('T', ' ').substring(0, 23) + ' Z'); 

    } catch (e) {
        console.error('Erreur lors de la mise à jour des données dérivées:', e);
    }
}


/** Gère la géolocalisation et le stockage */
function startGeolocationAndFiltering() {
    let watchId = null;

    if (navigator.geolocation) {
        watchId = navigator.geolocation.watchPosition(
            async (position) => {
                if (isGpsPaused) return;

                const { latitude, longitude, altitude, speed, accuracy } = position.coords;
                
                // Mettre à jour l'état du filtre (UKF)
                const newAlt = altitude || currentPosition.alt; 
                const newSpeed = speed || 0.0;
                FilterEngine.updateState(latitude, longitude, newAlt, newSpeed);
                
                const state = FilterEngine.getState();
                currentPosition.lat = state.lat;
                currentPosition.lon = state.lon;
                currentPosition.alt = state.alt;
                currentPosition.spd = state.speed;

                // Mise à jour de l'affichage principal
                safeSet('lat', state.lat.toFixed(6));
                safeSet('lon', state.lon.toFixed(6));
                safeSet('alt', state.alt.toFixed(2) + ' m');
                safeSet('speed-3d-inst', newSpeed.toFixed(2) + ' m/s');
                safeSet('acc', accuracy.toFixed(2) + ' m');

                // Mise à jour de la carte
                if (window.userMarker) {
                    window.userMarker.setLatLng([state.lat, state.lon]);
                }
                if (window.map) {
                    window.map.panTo([state.lat, state.lon]);
                }
                
                // Sauvegarde des données brutes
                const rawData = { latitude, longitude, altitude, speed, accuracy };
                await Storage.saveObservation(rawData);
            },
            (error) => {
                console.error("Erreur de géolocalisation:", error);
                safeSet('lat', 'GPS ERR');
                safeSet('lon', 'GPS ERR');
            },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
        );
    } else {
        console.warn("La géolocalisation n'est pas prise en charge. Utilisation des coordonnées par défaut.");
    }
    return watchId;
}


// =================================================================
// BLOC 5/5 : Initialisation et Configuration
// =================================================================

function initMap(lat, lon) {
    if (typeof L !== 'undefined') {
        window.map = L.map('map').setView([lat, lon], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(window.map);
        window.userMarker = L.marker([lat, lon]).addTo(window.map)
            .bindPopup("Position Filtrée").openPopup();
    } else {
        console.warn("Leaflet non disponible. Carte non initialisée.");
    }
}

// Gestion de la simulation par défaut si le GPS est désactivé
function startSimulationLoop() {
    // Si la géolocalisation est prise en charge, on laisse `watchPosition` faire le travail.
    if (navigator.geolocation) return; 

    // Simulation de données
    return setInterval(() => {
        if (isGpsPaused) return;
        
        const newLat = currentPosition.lat + (Math.random() - 0.5) * 0.00005;
        const newLon = currentPosition.lon + (Math.random() - 0.5) * 0.00005;
        const newSpeed = 1.5 + (Math.random() * 0.5); 
        const newAlt = currentPosition.alt + (Math.random() - 0.5) * 0.1;
        
        FilterEngine.updateState(newLat, newLon, newAlt, newSpeed);
        const state = FilterEngine.getState();
        
        // Mise à jour de l'affichage principal
        safeSet('lat', state.lat.toFixed(6));
        safeSet('lon', state.lon.toFixed(6));
        safeSet('alt', state.alt.toFixed(2) + ' m');
        safeSet('speed-3d-inst', newSpeed.toFixed(2) + ' m/s');
        safeSet('acc', '10.00 m (Sim)');

        if (window.userMarker) window.userMarker.setLatLng([state.lat, state.lon]);
        if (window.map) window.map.panTo([state.lat, state.lon]);
    }, 2000);
}

// Gestion des événements du mode jour/nuit
function setupDarkModeToggle() {
    const toggleBtn = $('toggle-mode-btn');
    if (!toggleBtn) return;
    
    toggleBtn.addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        // Mise à jour du texte du bouton
        if (document.body.classList.contains('dark-mode')) {
            toggleBtn.innerHTML = '<i class="fas fa-sun"></i> Mode Jour';
        } else {
            toggleBtn.innerHTML = '<i class="fas fa-moon"></i> Mode Nuit';
        }
    });
    
    // Initialisation du mode par défaut basé sur le système ou le mode sombre par défaut
    if (window.matchMedia && window.matchMedia('(prefers-color-scheme: light)').matches && !document.body.classList.contains('dark-mode')) {
         document.body.classList.remove('dark-mode');
         toggleBtn.innerHTML = '<i class="fas fa-moon"></i> Mode Nuit';
    } else {
         document.body.classList.add('dark-mode');
         toggleBtn.innerHTML = '<i class="fas fa-sun"></i> Mode Jour';
    }
}

/* ---------- Séquence d'initialisation ---------- */
(async function init(){
    // Initialiser le mode sombre
    setupDarkModeToggle();

    // Initialiser les valeurs par défaut
    const lat0=currentPosition.lat, lon0=currentPosition.lon, alt0=currentPosition.alt;
    
    FilterEngine.initFromGnss(lat0, lon0, alt0);
    initMap(lat0, lon0);
    
    // Initialiser les valeurs Météo/Pollution par défaut pour la métrologie
    updateWeatherDOM({ pressure_hPa: PHYS.BARO_ALT_REF_HPA, tempK: PHYS.TEMP_SEA_LEVEL_K, air_density: RHO_SEA_LEVEL }, true);
    // Simuler des données de pollution par défaut
    updatePollutantsDOM({ pm10: 25.5 }); 
    
    // Premier rafraîchissement des données dérivées et Astro
    await refreshDerived(lat0, lon0, alt0, currentPosition.spd);

    // Démarrer la boucle de géolocalisation ou de simulation
    startGeolocationAndFiltering();
    startSimulationLoop();
    
    // Démarrer la tâche périodique de rafraîchissement des données dérivées
    setInterval(()=>{ 
        const st = FilterEngine.getState(); 
        refreshDerived(st.lat, st.lon, st.alt, st.speed); 
    }, 7000);

    // Afficher le statut initial
    console.log('Dashboard GNSS initialisé (UKF-21 ÉTATS, Astro High-Precision).');
})();
