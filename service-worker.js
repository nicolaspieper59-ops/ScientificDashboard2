// =================================================================
// FICHIER service-worker.js (POUR FONCTIONNEMENT HORS LIGNE)
// =================================================================
const CACHE_NAME = 'dashboard-v3-cache';
// Liste des fichiers essentiels à mettre en cache
const urlsToCache = [
    '/',
    'index.html',
    'dashboard.js',
    'dashboard.css', // Assurez-vous d'avoir un fichier CSS avec ce nom
    // Vous pouvez ajouter d'autres ressources si nécessaire (images, polices)
];

// Événement 'install' : met en cache toutes les ressources
self.addEventListener('install', event => {
    event.waitUntil(
        caches.open(CACHE_NAME)
            .then(cache => {
                console.log('Service Worker: Cache ouvert, ajout des fichiers.');
                // L'option {cache: 'reload'} assure que le contenu est bien frais lors de l'installation
                return cache.addAll(urlsToCache.map(url => new Request(url, {cache: 'reload'})));
            })
    );
});

// Événement 'fetch' : intercepte les requêtes réseau et sert le cache en priorité
self.addEventListener('fetch', event => {
    event.respondWith(
        caches.match(event.request)
            .then(response => {
                // Si la ressource est dans le cache, la retourner
                if (response) {
                    return response;
                }
                // Sinon, la chercher sur le réseau (si l'utilisateur est en ligne)
                return fetch(event.request);
            })
            // Pour les requêtes API (météo) : si hors ligne, ça retournera l'erreur gérée dans dashboard.js
            .catch(error => {
                console.log('Service Worker: Erreur de fetch (probablement hors ligne) :', error);
                // Si la requête échoue et qu'elle n'est pas dans le cache, 
                // on pourrait retourner une page/ressource de secours (non implémenté ici)
            })
    );
});

// Événement 'activate' : nettoie les anciens caches
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
