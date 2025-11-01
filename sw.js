// =================================================================
// FICHIER Service Worker (sw.js) - Cache pour mode hors ligne
// =================================================================

const CACHE_NAME = 'gnss-dashboard-v12-0';

// Liste des fichiers essentiels à mettre en cache lors de l'installation
const ASSETS_TO_CACHE = [
    // Fichiers de la racine (Dashboard)
    './', // Représente index.html
    'index.html',
    './style.css', // Assurez-vous que votre fichier CSS a ce nom
    './gnss-dashboard-full.js',
    
    // Librairies externes (téléchargées localement)
    './leaflet.css',
    './leaflet.js',
    './suncalc.js',
    // Si vous utilisez FontAwesome localement (requis pour les emojis/icônes)
    './fontawesome.min.css', 
    // Assurez-vous d'inclure les polices de FontAwesome si elles sont utilisées
    // Exemple : './webfonts/fa-solid-900.woff2', 
];

// Étape 1: Installation du Service Worker et mise en cache des fichiers statiques
self.addEventListener('install', event => {
    console.log('[Service Worker] Installation...');
    event.waitUntil(
        caches.open(CACHE_NAME)
            .then(cache => {
                console.log('[Service Worker] Mise en cache des ressources...');
                return cache.addAll(ASSETS_TO_CACHE);
            })
            .catch(err => {
                console.error('[Service Worker] Échec de la mise en cache:', err);
            })
     
