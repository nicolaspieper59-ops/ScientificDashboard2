const CACHE_NAME = 'gnss-dashboard-v1.0.1'; // Changer la version pour forcer la mise à jour
const RESOURCES_TO_CACHE = [
    '/', // L'index racine
    '/index.html',
    '/gnss-dashboard-part1.js',
    '/gnss-dashboard-part2.js',
    '/suncalc.js',
    '/manifest.json',
    // Ajoutez ici les chemins de vos icônes
    '/icons/icon-192x192.png',
    '/icons/icon-512x512.png',
    // Les ressources CDN (important)
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
    'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0-beta3/css/all.min.css'
];

self.addEventListener('install', (event) => {
    event.waitUntil(
        caches.open(CACHE_NAME)
            .then((cache) => {
                console.log('Cache ouvert: Mise en cache des ressources.');
                return cache.addAll(RESOURCES_TO_CACHE);
            })
            .catch(err => {
                console.error('Échec de la mise en cache :', err);
            })
    );
});

self.addEventListener('fetch', (event) => {
    event.respondWith(
        caches.match(event.request)
            .then((response) => {
                // Retourne la ressource en cache si elle existe
                if (response) {
                    return response;
                }
                
                // Sinon, fetch (télécharge) la ressource
                return fetch(event.request);
            })
    );
});

self.addEventListener('activate', (event) => {
    const cacheWhitelist = [CACHE_NAME];
    event.waitUntil(
        caches.keys().then((cacheNames) => {
            return Promise.all(
                cacheNames.map((cacheName) => {
                    if (cacheWhitelist.indexOf(cacheName) === -1) {
                        return caches.delete(cacheName); // Supprime les vieux caches
                    }
                })
            );
        })
    );
});
