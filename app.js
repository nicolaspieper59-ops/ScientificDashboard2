// app.js (Extrait des CONSTANTES - Ajout de speed-stable)
// ...
// --- REFERENCES DOM ---
// ...
// NOUVEAU CHAMP DANS LE DOM
const speedStable = document.getElementById('speed-stable');
// ...

// app.js (Extrait de resetDisplay)
function resetDisplay() {
    // ...
    const defaultText = '--';
    const ids = ['elapsed-time', 'speed-3d-inst', 'speed-raw-ms', 'speed-avg', 'speed-max', 'speed-ms', 
        'perc-light', 'perc-sound', 'distance-km-m', 'lunar-time',
        'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground',
        'solar-true', 'solar-mean', 'eot', 'solar-longitude-val', 
        'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 
        'boiling-point', 'heading', 'bubble-level', 'cap-dest', 'solar-true-header', 
        'mode-indicator', 'speed-source-indicator', 'speed-stable' // AJOUT DU NOUVEL ID
    ];
    // ...
}
