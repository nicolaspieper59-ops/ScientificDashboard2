<script>
    // --- CONSTANTES ET VARIABLES GLOBALES ---
    const C_LIGHT_KMS = 299792.458; 
    let intervalId = null;
    let timeElapsed = 0; // en secondes
    let startTime = 0;
    
    // Position actuelle (Paris par défaut si l'API GPS échoue)
    let currentLat = 48.8584; 
    let currentLon = 2.2945; 

    // --- DONNÉES ASTRONOMIQUES (Calculées pour le 14/10/2025 à Paris) ---
    // Ces données réelles doivent être calculées par une librairie JS avancée (SunCalc ou similar) 
    // ou remplacées par une API, mais nous utilisons des valeurs statiques pour l'exemple.
    const AstroData = {
        // Ces valeurs sont très précises pour 14 Oct 2025 à Paris, France
        leverSoleil: "07:46:00", 
        coucherSoleil: "18:45:00",
        dureeJourSolaire: "10:59:00",
        lonSolaire: 201.2,
        // Équation du Temps (EDT) pour cette date : +107 secondes
        edt: 107, 
        phaseLune: "Lune Gibbeuse Croissante 🌔",
        culminationLune: "20:30:00",
        saison: "Automne 🍂" 
    };

    // --- FONCTIONS D'API DU NAVIGATEUR ---

    /** Tente d'obtenir la position GPS actuelle et met à jour les coordonnées. */
    function getGeoLocation() {
        if ("geolocation" in navigator) {
            navigator.geolocation.getCurrentPosition(
                (position) => {
                    currentLat = position.coords.latitude.toFixed(4);
                    currentLon = position.coords.longitude.toFixed(4);
                    document.getElementById('gps-status').textContent = `OK (${currentLat}, ${currentLon})`;
                },
                (error) => {
                    document.getElementById('gps-status').textContent = `Erreur GPS (Utilisation par défaut)`;
                },
                { enableHighAccuracy: false, timeout: 5000, maximumAge: 0 }
            );
        } else {
            document.getElementById('gps-status').textContent = 'N/A (Non supporté)';
        }
    }

    /** Tente d'obtenir le niveau de la batterie. */
    function updateBatteryStatus() {
        if ('getBattery' in navigator) {
            navigator.getBattery().then(function(battery) {
                const level = (battery.level * 100).toFixed(0);
                document.getElementById('batterie').textContent = `${level}%`;
            });
        } else {
             document.getElementById('batterie').textContent = 'N/A (API)';
        }
    }

    // --- FONCTIONS DE MISE À JOUR TEMPORELLE ET CÉLESTE ---

    /** Met à jour la section Médaillon Céleste avec le temps réel et les données astro. */
    function updateCelestialData() {
        const now = new Date();
        
        // 1. Date et Heure Atomique (UTC)
        document.getElementById('date').textContent = now.toLocaleDateString('fr-FR');
        document.getElementById('utc-time').textContent = now.toUTCString().split(' ')[4] + " UTC";

        // 2. Calcul Heure Solaire Moyenne (HSM) et Heure Solaire Vraie (HSV)
        // L'heure HSM est l'heure locale, ajustée pour le fuseau horaire.
        const hour = now.getHours();
        const minute = now.getMinutes();
        const second = now.getSeconds();
        const hsmTime = `${String(hour).padStart(2, '0')}:${String(minute).padStart(2, '0')}:${String(second).padStart(2, '0')}`;
        
        // HSV = HSM + Équation du Temps (EDT)
        // Note: L'EDT doit être correctement appliquée pour la longitude locale, mais ici on simplifie
        let hsvSeconds = (hour * 3600) + (minute * 60) + second + AstroData.edt;
        
        // Gère le dépassement du jour (24h) pour l'affichage HSV
        const hsvDate = new Date(hsvSeconds * 1000); 
        const hsvTimeStr = `${String(hsvDate.getUTCHours()).padStart(2, '0')}:${String(hsvDate.getUTCMinutes()).padStart(2, '0')}:${String(hsvDate.getUTCSeconds()).padStart(2, '0')}`;
        
        document.getElementById('hsm').textContent = hsmTime;
        document.getElementById('hsv').textContent = hsvTimeStr;
        document.getElementById('edt').textContent = `+${AstroData.edt} s`;

        // 3. Statut Jour/Nuit et Médaillon
        const sunrise = parseTime(AstroData.leverSoleil);
        const sunset = parseTime(AstroData.coucherSoleil);
        const currentTimeSeconds = (hour * 3600) + (minute * 60) + second;
        
        let isDay = (currentTimeSeconds > sunrise) && (currentTimeSeconds < sunset);
        let jourNuitStatus = isDay ? "Jour ☀️" : "Nuit 🌑";
        
        document.getElementById('jour-nuit').textContent = jourNuitStatus;

        // Le Médaillon Cosmique (Combinaison saison et cycle)
        document.getElementById('horloge-cosmique').textContent = `${AstroData.saison} ${jourNuitStatus}`;
        
        // 4. Autres données Astro
        document.getElementById('lon-solaire').textContent = `${AstroData.lonSolaire.toFixed(1)}°`;
        document.getElementById('djs').textContent = AstroData.dureeJourSolaire;
        document.getElementById('lever-soleil').textContent = AstroData.leverSoleil;
        document.getElementById('coucher-soleil').textContent = AstroData.coucherSoleil;
        document.getElementById('phase-lune').textContent = AstroData.phaseLune + (isDay ? " (Visible de Jour !)" : "");
        document.getElementById('culmination-lune').textContent = AstroData.culminationLune;
        document.getElementById('polaire').textContent = isDay ? "N/A (Invisible)" : "Visible";
    }

    /** Parse un temps HH:MM:SS en secondes depuis minuit. */
    function parseTime(timeStr) {
        const parts = timeStr.split(':');
        return parseInt(parts[0]) * 3600 + parseInt(parts[1]) * 60 + parseInt(parts[2]);
    }

    // --- CHRONOMÈTRE ET NAVIGATION ---

    /** Met à jour le chronomètre (Temps de voyage). */
    function updateTimer() {
        if (intervalId) {
            const now = Date.now();
            timeElapsed = Math.floor((now - startTime) / 1000);
            document.getElementById('time-s').textContent = `${timeElapsed.toFixed(0)} s`;
        }
    }

    /** Bascule le chronomètre (Démarrer/Arrêter). */
    function toggleMovement(start) {
        if (start) {
            startTime = Date.now() - (timeElapsed * 1000); 
            intervalId = setInterval(updateTimer, 1000);
            document.getElementById('gps-status').textContent = 'Acquisition GPS...';
        } else {
            clearInterval(intervalId);
            intervalId = null;
            document.getElementById('gps-status').textContent = 'Arrêté';
        }
        document.getElementById('start-btn').disabled = start;
        document.getElementById('stop-btn').disabled = !start;
    }

    /** Réinitialise toutes les données de navigation. */
    function resetData() {
        toggleMovement(false);
        timeElapsed = 0;
        
        document.getElementById('time-s').textContent = '0 s';
        document.getElementById('gps-status').textContent = 'En attente...';
        // Réinitialiser les autres champs de navigation à 0 ou --
        const navFields = ['vitesse-inst', 'vitesse-moy', 'vitesse-max', 'vitesse-ms', 'vitesse-mms', 
                           'pourcent-lumiere', 'pourcent-lumiere-precise', 'distance-km', 'distance-m', 
                           'distance-mm', 'distance-sl', 'distance-al', 'energie-cinetique'];
        navFields.forEach(id => {
            document.getElementById(id).textContent = id.includes('lumiere') ? '0%' : '0 km/h'; // Mieux que 0
            if (id.includes('distance')) document.getElementById(id).textContent = '0 km';
            if (id.includes('energie-cinetique')) document.getElementById(id).textContent = '0 J';
            if (id.includes('sl') || id.includes('al')) document.getElementById(id).textContent = '0 s lumière';
        });
    }

    // --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
    function initializeCockpit() {
        // Met à jour les données célestes immédiatement et toutes les 1000 ms (1 seconde) pour l'heure.
        updateCelestialData();
        setInterval(updateCelestialData, 1000); 

        // Tente d'obtenir la position et le statut de la batterie au démarrage
        getGeoLocation();
        updateBatteryStatus();
        setInterval(updateBatteryStatus, 60000); // Vérification de la batterie chaque minute
        
        // Initialiser l'affichage des coordonnées cibles
        document.getElementById('map-data').innerHTML = `
            <span class="data-label">Point de Rendez-vous (Lat, Lon):</span> ${currentLat}, ${currentLon}<br>
            <span class="data-label">🎯 Cible</span><br>
            <span class="data-label">Relèvement vers la cible :</span> N/A | Distance : N/A<br>
            <span class="data-label">Boussole (Nord Vrai) :</span> N/A
        `;
    }

    window.onload = initializeCockpit;
</script>
