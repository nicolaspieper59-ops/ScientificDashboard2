// =================================================================
// FICHIER service-worker.js (POUR FONCTIONNEMENT HORS LIGNE/PWA)
// =================================================================
const CACHE_NAME = 'dashboard-v4-offline-cache';
// Liste des fichiers essentiels à mettre en cache
const urlsToCache = [
    '/',
    'index.html',
    'dashboard.js',
    // Ces URLs sont pour des ressources externes, mais sont généralement 
    // mises en cache par le Service Worker s'ils sont chargés la première fois.
    // Pour une conformité PWA stricte, il faut aussi mettre en cache ces dépendances
    'https://cdnjs.cloudflare.com/ajax/libs/html2canvas/1.4.1/html2canvas.min.js',
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
    'manifest.json'
];

// Événement 'install' : met en cache toutes les ressources
self.addEventListener('install', event => {
    console.log('SW: Installation. Mise en cache des ressources essentielles.');
    self.skipWaiting();
    event.waitUntil(
        caches.open(CACHE_NAME)
            .then(cache => {
                // Utiliser {cache: 'reload'} pour s'assurer que le contenu est bien frais lors de l'installation
                return cache.addAll(urlsToCache.map(url => new Request(url, {cache: 'reload'})))
                    .catch(error => {
                        console.warn('SW: Certaines ressources externes (CDN) n\'ont pas pu être mises en cache.', error);
                        // Permettre la poursuite même si certaines ressources CDN échouent
                        return Promise.resolve(); 
                    });
            })
    );
});

// Événement 'fetch' : sert le cache d'abord, puis le réseau
self.addEventListener('fetch', event => {
    // Ne pas intercepter les requêtes non-GET ou les requêtes d'extensions
    if (event.request.method !== 'GET' || !event.request.url.startsWith('http')) {
        return;
    }

    event.respondWith(
        caches.match(event.request)
            .then(response => {
                // Si la ressource est dans le cache, la retourner immédiatement
                if (response) {
                    return response;
                }
                
                // Sinon, essayer le réseau
                return fetch(event.request).catch(() => {
                    // Si le réseau échoue (hors ligne) et que ce n'est pas dans le cache,
                    // on peut retourner une réponse par défaut ou laisser l'application gérer l'erreur (ex: API météo)
                    console.log('SW: Requête réseau échouée pour:', event.request.url);
                });
            })
    );
});

// Événement 'activate' : nettoie les anciens caches
self.addEventListener('activate', event => {
    console.log('SW: Activation. Nettoyage des anciens caches.');
    const cacheWhitelist = [CACHE_NAME];
    event.waitUntil(
        caches.keys().then(cacheNames => {
            return Promise.all(
                cacheNames.map(cacheName => {
                    if (cacheWhitelist.indexOf(cacheName) === -1) {
                        return caches.delete(cacheName);
                    }
                })
            ).then(() => self.clients.claim()); 
        })
    );
});
