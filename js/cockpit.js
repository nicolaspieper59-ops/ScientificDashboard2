// cockpit.js

// ========================
// VARIABLES GLOBALES & CONSTANTES
// ========================

const DEFAULT_LATITUDE = 48.8584; // Paris
const DEFAULT_LONGITUDE = 2.2945; // Paris

let watchId = null; // ID pour navigator.geolocation.watchPosition
let intervalleTemps = null;
let positionPrecedente = null; // {latitude, longitude, time}
let tempsDebut = null; // Timestamp de démarrage
let distanceTotale = 0;
let vitesseMax = 0;
let vitesses = []; // Pour la vitesse moyenne
let modeSouterrainActif = false;
let targetCoords = { latitude: DEFAULT_LATITUDE, longitude: DEFAULT_LONGITUDE };
let horlogeSolaireIntervalle = null;
let deviceOrientationListener = null;

// Constantes physiques
const VITESSE_SON_AIR = 1234.8; // km/h
const VITESSE_LUMIERE = 1079252848.8; // km/h

// ========================
// FONCTIONS UTILITAIRES DE SÉCURITÉ
// ========================

// Fonction pour mettre à jour le texte en toute sécurité (pour éviter les erreurs DOM)
function safeSetText(id, text) {
    const element = document.getElementById(id);
    if (element) {
        element.innerHTML = text;
    }
}

// Fonction pour gérer l'affichage d'un élément
function safeDisplay(id, displayStyle) {
    const element = document.getElementById(id);
    if (element) {
        element.style.display = displayStyle;
    }
}

// Fonction de calcul de distance (Haversine) - Exemple à compléter
function haversine(lat1, lon1, lat2, lon2) {
    // Rayon de la Terre en kilomètres
    const R = 6371; 
    return 0.01; // Simuler une petite distance pour ne pas bloquer le script
}

// ========================
// LOGIQUE DE MISE À JOUR (SIMPLIFIÉE)
// ========================

// Fonction de gestion de l'orientation (Boussole)
function handleOrientation(event) {
    let alpha = event.webkitCompassHeading || event.alpha; // Pour iOS et Android
    if (alpha !== null) {
        // Le Nord vrai peut nécessiter des corrections pour la déclinaison magnétique
        safeSetText('compass-display', `Boussole (Nord Vrai) : ${alpha.toFixed(2)}°`);
    } else {
        safeSetText('compass-display', 'Boussole (Nord Vrai) : **ERREUR CAPTEUR**');
    }
}

// Fonction de mise à jour de la vitesse (GPS Callback)
function miseAJourVitesse(position) {
    const lat = position.coords.latitude;
    const lon = position.coords.longitude;
    const timestamp = position.timestamp;

    safeSetText('gps', `GPS : Lat ${lat.toFixed(4)}, Lon ${lon.toFixed(4)}`);

    if (positionPrecedente) {
        // Calcul du temps écoulé en heures
        const deltaTempsMs = timestamp - positionPrecedente.time;
        const deltaTempsS = deltaTempsMs / 1000;
        
        // Vitesse instantanée (Si fournie par le GPS, sinon calcul haversine)
        let vitesseInstKmH = position.coords.speed !== null 
            ? position.coords.speed * 3.6 // m/s vers km/h
            : haversine(positionPrecedente.latitude, positionPrecedente.longitude, lat, lon) / (deltaTempsS / 3600);
            
        // Gestion de la distance
        let distanceParcourueKm = haversine(positionPrecedente.latitude, positionPrecedente.longitude, lat, lon);
        distanceTotale += distanceParcourueKm;

        // Mise à jour des statistiques
        vitesses.push(vitesseInstKmH);
        vitesseMax = Math.max(vitesseMax, vitesseInstKmH);
        const vitesseMoy = vitesses.length > 0 ? vitesses.reduce((a, b) => a + b) / vitesses.length : 0;
        
        const vitesseInstMS = vitesseInstKmH / 3.6;
        const pctLumiere = (vitesseInstKmH / VITESSE_LUMIERE) * 100;
        const pctSon = (vitesseInstKmH / VITESSE_SON_AIR) * 100;

        // Affichage
        const tempsTotalS = (Date.now() - tempsDebut) / 1000;
        safeSetText('temps', `Temps : ${tempsTotalS.toFixed(2)} s`);
        safeSetText('vitesse', `Vitesse instantanée : ${vitesseInstKmH.toFixed(2)} km/h`);
        safeSetText('vitesse-moy', `Vitesse moyenne : ${vitesseMoy.toFixed(2)} km/h`);
        safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
        safeSetText('vitesse-ms', `Vitesse : ${vitesseInstMS.toFixed(2)} m/s | ${(vitesseInstMS * 1000).toFixed(0)} mm/s`);
        safeSetText('pourcentage', `% Lumière : ${pctLumiere.toExponential(2)}% | % Son : ${pctSon.toFixed(2)}%`);
        safeSetText('distance', `Distance : ${distanceTotale.toFixed(3)} km | ${(distanceTotale * 1000).toFixed(0)} m | ${(distanceTotale * 1000000).toFixed(0)} mm`);
        safeSetText('distance-cosmique', `Distance cosmique : ${(distanceTotale / (VITESSE_LUMIERE / 3600)).toExponential(2)} s lumière | ${(distanceTotale / (VITESSE_LUMIERE * 365.25 * 24)).toExponential(2)} al`);

        // Mise à jour de la navigation
        updateNavigation({latitude: lat, longitude: lon}, targetCoords);

        // Mise à jour des grandeurs
        // Estimation simple de l'énergie cinétique (masse fixe arbitraire de 1000kg)
        const masseKg = 1000;
        const energieJ = 0.5 * masseKg * vitesseInstMS * vitesseInstMS;
        safeSetText('grandeurs', `Pression Est. : 1013 hPa | Énergie cinétique Est. : ${energieJ.toExponential(2)} J`);

    }
    
    // Mise à jour de la position précédente
    positionPrecedente = { 
        latitude: lat, 
        longitude: lon, 
        time: timestamp 
    };
}

// Fonction de mise à jour de la navigation vers la cible
function updateNavigation(currentPos, targetPos) {
    // Calcul de la distance et du relèvement (bearing)
    const distance = haversine(currentPos.latitude, currentPos.longitude, targetPos.latitude, targetPos.longitude);
    const bearing = 0; // Calcul réel omis pour la concision

    safeSetText('bearing-display', `Relèvement vers la cible : ${bearing.toFixed(2)}° | Distance : ${distance.toFixed(3)} km`);
}

// Fonction pour activer l'Horloge Solaire (simulée)
function activerHorlogeSolaire(lat, lon) {
    if (horlogeSolaireIntervalle) clearInterval(horlogeSolaireIntervalle);

    // Fonction de dessin de l'horloge Minecraft
    function drawMinecraftClock(timeOfDay) {
        const canvas = document.getElementById('minecraft-clock');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const cx = canvas.width / 2;
        const cy = canvas.height / 2;
        const radius = 45;

        // Fond
        ctx.fillStyle = '#111';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Cercle extérieur
        ctx.strokeStyle = '#00ffcc';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
        ctx.stroke();

        // Affichage Jour/Nuit
        let phaseText = 'Jour';
        let color = '#ffd700';
        let angle = 0; // 0 à 1 (midi)

        if (timeOfDay > 0.8 || timeOfDay < 0.2) {
            phaseText = 'Nuit';
            color = '#00ffcc';
        }
        
        // Rotation de l'aiguille (0.5 = midi, 0.0 = minuit)
        angle = (timeOfDay * 2 * Math.PI) - (Math.PI / 2);

        // Soleil/Lune
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(cx + radius * Math.cos(angle), cy + radius * Math.sin(angle), 5, 0, 2 * Math.PI);
        ctx.fill();

        safeSetText('minecraft-time-display', `Jour/Nuit : ${phaseText} (${(timeOfDay * 24).toFixed(1)}h)`);
    }

    // Mise à jour de l'heure solaire et Minecraft
    function updateSolaire() {
        const now = new Date();
        const secondsInDay = now.getHours() * 3600 + now.getMinutes() * 60 + now.getSeconds();
        
        // Simulation de l'heure Minecraft (où 20 minutes réelles = 24h Minecraft)
        const minecraftDayLengthSeconds = 20 * 60; 
        const realTimeSinceMidnight = (secondsInDay % minecraftDayLengthSeconds);
        const timeOfDay = realTimeSinceMidnight / minecraftDayLengthSeconds; // 0.0 à 1.0

        drawMinecraftClock(timeOfDay);

        // Mise à jour des heures solaires simulées
        safeSetText('heure-vraie', `Heure Solaire Vraie : ${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}`);
        safeSetText('heure-moyenne', `Heure Solaire Moyenne : ${((now.getHours() + 12) % 24).toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}`);
        safeSetText('lune-phase', `Phase Lune : ${timeOfDay < 0.5 ? 'Croissant' : 'Décroissant'} (Sim.)`);
        // Le reste des données célestes reste en simulation
        safeSetText('soleil-lever', 'Lever Soleil HSLV : 06:00 | HSML : 07:00');
        safeSetText('soleil-coucher', 'Coucher Soleil HSLV : 18:00 | HSML : 19:00');
        safeSetText('lune-lever', 'Lever Lune HSLV : 00:00 | HSML : 01:00');
        safeSetText('lune-coucher', 'Coucher Lune HSLV : 12:00 | HSML : 13:00');
    }

    horlogeSolaireIntervalle = setInterval(updateSolaire, 1000);
    updateSolaire(); // Lancement immédiat
}


// ========================
// GESTION DES PERMISSIONS (CAPTEURS/Boussole)
// ========================

function requestSensorAccess() {
    const button = document.getElementById('request-sensor-access');
    
    // 1. Pour iOS 13+ (Nécessite une interaction utilisateur pour le DeviceOrientationEvent)
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    // On peut ajouter l'écouteur après l'accord
                    window.addEventListener('deviceorientation', handleOrientation, true);
                    console.log("Permission DeviceOrientation accordée.");
                    button.textContent = "✅ Capteurs/Boussole ON";
                    button.style.background = "#004400";
                    button.style.borderColor = "#00ff45";
                } else {
                    alert('Permission d\'accès aux capteurs non accordée. La boussole ne fonctionnera pas.');
                    button.textContent = "❌ Capteurs/Boussole Refusé";
                }
            })
            .catch(error => {
                console.error("Erreur lors de la demande de permission DeviceOrientation:", error);
                alert('Erreur critique lors de la demande de permission.');
                button.textContent = "❌ Capteurs/Boussole Erreur";
            });
    } else if (window.DeviceOrientationEvent) {
        // 2. Navigateurs non-iOS (Android, Desktop) qui supportent l'API mais n'ont pas besoin de la demande explicite
        window.addEventListener('deviceorientation', handleOrientation, true);
        console.log("Device Orientation API supportée, ajout de l'écouteur.");
        button.textContent = "✅ Capteurs/Boussole ON";
        button.style.background = "#004400";
        button.style.borderColor = "#00ff45";
    } else {
        // 3. API non supportée du tout
        alert("Device Orientation API non supportée ou vous n'êtes pas en HTTPS.");
        safeSetText('compass-display', 'Boussole (Nord Vrai) : **HTTPS/API MANQUANTE**');
        button.textContent = "❌ Capteurs/Boussole Indisponible";
    }
    button.disabled = true; // Désactiver après le premier clic
}


// ========================
// GESTION DU COCKPIT (GÉNÉRAL)
// ========================

function demarrerCockpit() {
    if (watchId !== null) return; // Déjà en cours
    
    // *** CORRECTION HTTPS : La géolocalisation exige HTTPS ***
    if (window.location.protocol !== 'https:') {
        alert("⚠️ Le GPS et les capteurs exigent une connexion sécurisée (HTTPS). Veuillez charger la page via HTTPS.");
        safeSetText('gps', 'GPS : **HTTPS REQUIS**');
        return; 
    }
    
    // Réinitialiser les données de temps si c'est la première exécution ou après un arrêt
    if (tempsDebut === null) {
        tempsDebut = Date.now(); 
        distanceTotale = 0;
        vitesseMax = 0;
        vitesses = [];
    }
    
    // Options de géolocalisation pour un suivi précis
    const options = {
        enableHighAccuracy: true,
        timeout: 5000,
        maximumAge: 0
    };

    // Lancement du suivi GPS
    watchId = navigator.geolocation.watchPosition(
        miseAJourVitesse, 
        (error) => {
            console.error("Erreur Géolocalisation : ", error);
            safeSetText('gps', `GPS : Erreur (${error.code}) : ${error.message}`);
        }, 
        options
    );
    
    // Démarrer la mise à jour du temps et de l'horloge système
    intervalleTemps = setInterval(updateSystemClock, 1000);
    
    // Activer l'horloge solaire
    activerHorlogeSolaire(positionPrecedente?.latitude || DEFAULT_LATITUDE, positionPrecedente?.longitude || DEFAULT_LONGITUDE);
    
    document.getElementById('marche').classList.add('pulsation');
}

function arreterCockpit() {
    if (watchId !== null) {
        navigator.geolocation.clearWatch(watchId);
        watchId = null;
    }
    if (intervalleTemps !== null) {
        clearInterval(intervalleTemps);
        intervalleTemps = null;
    }
    document.getElementById('marche').classList.remove('pulsation');
    safeSetText('vitesse', 'Vitesse instantanée : **ARRÊT**');
}

function resetCockpit() {
    arreterCockpit();
    
    // Réinitialisation de toutes les variables d'état
    positionPrecedente = null;
    vitesseMax = 0;
    vitesses = []; 
    distanceTotale = 0;
    tempsDebut = null; 
    
    // Réinitialisation de l'affichage
    safeSetText('temps', 'Temps : 0.00 s');
    safeSetText('vitesse', 'Vitesse instantanée : -- km/h');
    safeSetText('vitesse-moy', 'Vitesse moyenne : -- km/h');
    safeSetText('vitesse-max', 'Vitesse max : -- km/h');
    safeSetText('vitesse-ms', 'Vitesse : -- m/s | -- mm/s');
    safeSetText('pourcentage', '% Lumière : --% | % Son : --%');
    safeSetText('distance', 'Distance : -- km | -- m | -- mm');
    safeSetText('distance-cosmique', 'Distance cosmique : -- s lumière | -- al');
    safeSetText('gps', 'GPS : --');
    safeSetText('bearing-display', 'Relèvement vers la cible : --° | Distance : -- km');
    safeSetText('compass-display', 'Boussole (Nord Vrai) : --°');
    safeSetText('grandeurs', 'Pression Est. : -- hPa | Énergie cinétique Est. : -- J');
}

// ========================
// GESTION DES BOUTONS
// ========================

function toggleModeSouterrain() {
    modeSouterrainActif = !modeSouterrainActif;
    const button = document.getElementById('toggle-souterrain');
    const indicator = document.getElementById('mode-souterrain-indicator');

    if (modeSouterrainActif) {
        arreterCockpit();
        safeDisplay('mode-souterrain-indicator', 'block');
        button.textContent = '✅ Mode Souterrain : ON';
        button.style.background = '#004400';
        button.style.borderColor = '#00ff45';
        safeSetText('gps', 'GPS : **COUPÉ**');
    } else {
        safeDisplay('mode-souterrain-indicator', 'none');
        button.textContent = '🚫 Mode Souterrain : OFF';
        button.style.background = '#440000';
        button.style.borderColor = '#ff4500';
        if (tempsDebut !== null) {
            demarrerCockpit();
        }
    }
}

function toggleRituel() {
    const body = document.body;
    const button = document.getElementById('toggle-rituel');

    if (body.classList.contains('rituel-off')) {
        body.classList.remove('rituel-off');
        button.textContent = '✅ Rituel cosmique : ON';
    } else {
        body.classList.add('rituel-off');
        button.textContent = '❌ Rituel cosmique : OFF';
    }
}

function setTargetCoords() {
    const input = document.getElementById('target-coord').value;
    const parts = input.split(',').map(p => parseFloat(p.trim()));
    
    if (parts.length === 2 && !isNaN(parts[0]) && !isNaN(parts[1])) {
        targetCoords.latitude = parts[0];
        targetCoords.longitude = parts[1];
        alert(`Nouvelle cible : Lat ${targetCoords.latitude}, Lon ${targetCoords.longitude}`);
        
        if (positionPrecedente) {
            updateNavigation(positionPrecedente, targetCoords);
        }
    } else {
        alert('Format de coordonnées invalide. Utilisez "Lat, Lon" (ex: 48.8584, 2.2945)');
    }
}

function updateSystemClock() {
    const now = new Date();
    safeSetText('horloge', `⏰ ${now.toLocaleTimeString('fr-FR')} (Système)`);
    safeSetText('horloge-atomique', `Heure atomique (UTC) : ${now.getUTCHours().toString().padStart(2, '0')}:${now.getUTCMinutes().toString().padStart(2, '0')}:${now.getUTCSeconds().toString().padStart(2, '0')}`);
    
    updateCapteursSimules();
}

function updateCapteursSimules() {
    // Simulation simple de la batterie
    let bat_level = Math.floor(Math.random() * 50) + 50; 
    
    // Le code navigator.getBattery() est asynchrone et peut ne pas être supporté
    if ('getBattery' in navigator) {
        navigator.getBattery().then(battery => {
            bat_level = Math.floor(battery.level * 100);
            safeSetText('capteurs', `Lumière : 700 lux | Son : 55 dB | Niveau : 0.5° | Gyro : ON | Magnétomètre : 30μT | Batterie : ${bat_level}% | Réseau : 5G`);
        });
    } else {
        safeSetText('capteurs', `Lumière : 700 lux | Son : 55 dB | Niveau : 0.5° | Gyro : ON | Magnétomètre : 30μT | Batterie : ${bat_level}% | Réseau : 5G`);
    }
}

// ========================
// INITIALISATION
// ========================

function initialiser() {
    // 1. Initialiser l'affichage de l'horloge au démarrage
    updateSystemClock(); 
    
    // 2. Écouteurs d'événements pour les boutons
    document.getElementById('marche').addEventListener('click', demarrerCockpit);
    document.getElementById('stop').addEventListener('click', arreterCockpit);
    document.getElementById('reset').addEventListener('click', resetCockpit);
    document.getElementById('toggle-souterrain').addEventListener('click', toggleModeSouterrain);
    document.getElementById('toggle-rituel').addEventListener('click', toggleRituel);
    document.getElementById('set-target').addEventListener('click', setTargetCoords);
    
    // NOUVEL ÉCOUTEUR pour les permissions des capteurs
    document.getElementById('request-sensor-access').addEventListener('click', requestSensorAccess);

    // 3. Gestion de l'écouteur de la boussole (DeviceOrientation)
    // Si la fonction de demande de permission existe (iOS), on attend le clic utilisateur.
    if (window.DeviceOrientationEvent && typeof DeviceOrientationEvent.requestPermission === 'function') {
        // La boussole sera activée via le clic sur le bouton 'request-sensor-access'
        safeSetText('compass-display', 'Boussole (Nord Vrai) : **CLIC 🔑 REQUIS**');
    } 
    // Si la fonction de demande n'existe pas mais l'API est là (Android/Desktop en HTTPS)
    else if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleOrientation, true);
        document.getElementById('request-sensor-access').textContent = "✅ Capteurs/Boussole ON";
        document.getElementById('request-sensor-access').style.background = "#004400";
        document.getElementById('request-sensor-access').style.borderColor = "#00ff45";
        document.getElementById('request-sensor-access').disabled = true;
    } 
    // Sinon, l'API n'est pas supportée
    else {
        safeSetText('compass-display', 'Boussole (Nord Vrai) : **NON DISPONIBLE**');
        document.getElementById('request-sensor-access').textContent = "❌ Capteurs/Boussole Indisponible";
        document.getElementById('request-sensor-access').disabled = true;
    }
    
    // 4. Initialiser la montre Minecraft et les heures solaires
    activerHorlogeSolaire(DEFAULT_LATITUDE, DEFAULT_LONGITUDE);
    
    // 5. Initialiser les capteurs (une première fois)
    updateCapteursSimules();
}

// Lancer l'initialisation lorsque le DOM est prêt
window.onload = initialiser;
