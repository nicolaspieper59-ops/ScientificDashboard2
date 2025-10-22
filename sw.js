const CACHE_NAME = 'science-dash-v1';
const urlsToCache = [
    '/',
    '/index.html',
    '/dashboard.js',
    '/manifest.json'
];

// Installation du Service Worker et mise en cache des assets statiques
self.addEventListener('install', event => {
    event.waitUntil(
        caches.open(CACHE_NAME)
            .then(cache => {
                console.log('Service Worker installé. Fichiers mis en cache.');
                return cache.addAll(urlsToCache);
            })
    );
});

// Interception des requêtes : sert la version en cache ou fait une requête réseau
self.addEventListener('fetch', event => {
    event.respondWith(
        caches.match(event.request)
            .then(response => {
                // Fichier trouvé dans le cache
                if (response) {
                    return response;
                }
                // Aucun fichier en cache, requête réseau (y compris la synchro de l'heure)
                return fetch(event.request);
            })
    );
});

// Mise à jour : suppression des anciens caches
self.addEventListener('activate', event => {
    const cacheWhitelist = [CACHE_NAME];
    event.waitUntil(
        caches.keys().then(cacheNames => {
            return Promise.all(
                cacheNames.map(cacheName => {
                    if (cacheWhitelist.indexOf(cacheName) === -1) {
                        return caches.delete(cacheName);
                    }
                })
            );
        })
    );
});
