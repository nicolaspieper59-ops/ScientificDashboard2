// ... (Code JavaScript existant jusqu'ici) ...

// ========================
// GESTION DU COCKPIT (GÉNÉRAL)
// ========================

function demarrerCockpit() {
    if (watchId !== null) return; // Déjà en cours
    
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
    
    // Activer l'horloge solaire (elle gère son propre intervalle)
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
    // Le tempsDebut reste pour calculer le temps total de vol
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
        // En mode souterrain, on coupe le GPS et on affiche l'alerte
        arreterCockpit();
        safeDisplay('mode-souterrain-indicator', 'block');
        button.textContent = '✅ Mode Souterrain : ON';
        button.style.background = '#004400';
        button.style.borderColor = '#00ff45';
        
        safeSetText('gps', 'GPS : **COUPÉ**');
    } else {
        // En sortie de mode souterrain, on réactive le GPS
        safeDisplay('mode-souterrain-indicator', 'none');
        button.textContent = '🚫 Mode Souterrain : OFF';
        button.style.background = '#440000';
        button.style.borderColor = '#ff4500';
        
        // On relance le cockpit si ce n'était pas déjà à l'arrêt volontaire
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
        
        // Mise à jour immédiate de la navigation si la position est connue
        if (positionPrecedente) {
            updateNavigation(positionPrecedente, targetCoords);
        }
    } else {
        alert('Format de coordonnées invalide. Utilisez "Lat, Lon" (ex: 48.8584, 2.2945)');
    }
}

function updateSystemClock() {
    const now = new Date();
    // Affichage de l'horloge système
    safeSetText('horloge', `⏰ ${now.toLocaleTimeString('fr-FR')} (Système)`);
    // Affichage de l'heure atomique (UTC)
    safeSetText('horloge-atomique', `Heure atomique (UTC) : ${now.getUTCHours().toString().padStart(2, '0')}:${now.getUTCMinutes().toString().padStart(2, '0')}:${now.getUTCSeconds().toString().padStart(2, '0')}`);
    
    // Mettre à jour les capteurs simulés/simplifiés (Lumière, Batterie, etc.)
    updateCapteursSimules();
}

function updateCapteursSimules() {
    // Simulation simple de la batterie
    let bat_level = Math.floor(Math.random() * 50) + 50; // entre 50 et 100%
    if (navigator.getBattery) {
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

    // 3. Écouteur pour la boussole (DeviceOrientation)
    if (window.DeviceOrientationEvent) {
        deviceOrientationListener = window.addEventListener('deviceorientation', handleOrientation, true);
    } else {
        console.warn("Device Orientation API non supportée. Boussole non fonctionnelle.");
        safeSetText('compass-display', 'Boussole (Nord Vrai) : **NON DISPONIBLE**');
    }
    
    // 4. Initialiser la montre Minecraft et les heures solaires
    activerHorlogeSolaire(DEFAULT_LATITUDE, DEFAULT_LONGITUDE);
    
    // 5. Initialiser les capteurs (une première fois)
    updateCapteursSimules();
}

// Lancer l'initialisation lorsque le DOM est prêt
window.onload = initialiser;

// =====================================================================================
// FIN DU CODE JAVASCRIPT
// =====================================================================================
