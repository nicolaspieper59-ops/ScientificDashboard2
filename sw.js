// =================================================================
// FICHIER JS : sw.js (Service Worker)
// Gestion de la mise en cache pour PWA
// =================================================================

const CACHE_NAME = 'gnss-dashboard-cache-v1.0.0'; 

// Liste des fichiers essentiels à mettre en cache lors de l'installation
const urlsToCache = [
    '/',
    'index.html',
    'style.css',
    'fontawesome.min.css',
    'leaflet.css',
    'leaflet.js',
    'suncalc.js',
    'gnss-dashboard-full.js', 
    'manifest.json',
    'sw.js', 
    // Icônes de l'application (basé sur le nom mentionné)
    'icons/icon-192.png', 
    // Note: 'api/weather.js' n'est généralement pas mis en cache ici 
    // car il s'agit d'un script côté serveur ou d'une API de données.
];

// --- 1. Installation du Service Worker et Mise en Cache des Ressources ---
self.addEventListener('install', event => {
    // S'assure que le Service Worker ne sera pas actif tant que tous les fichiers ne sont pas mis en cache.
    event.waitUntil(
        caches.open(CACHE_NAME)
            .then(cache => {
                console.log('Service Worker: Fichiers de base mis en cache avec succès.');
                return cache.addAll(urlsToCache);
            })
            .catch(error => {
                console.error('Service Worker: Échec de la mise en cache lors de l\'installation:', error);
            })
    );
    // Force l'activation immédiate du nouveau Service Worker
    self.skipWaiting();
});

// --- 2. Interception des Requêtes et Stratégie Cache-First ---
self.addEventListener('fetch', event => {
    // Stratégie pour les fichiers mis en cache (CSS, JS, HTML, images) : Cache-First
    event.respondWith(
        caches.match(event.request)
            .then(response => {
                // Si la ressource est trouvée dans le cache, la retourner.
                if (response) {
                    return response;
                }
                
                // Sinon, effectuer la requête réseau.
                // Cloner la requête car une requête est un flux (stream) et ne peut être lue qu'une fois.
                const fetchRequest = event.request.clone();

                return fetch(fetchRequest).then(
                    networkResponse => {
                        // Vérifier si nous avons reçu une réponse valide
                        if(!networkResponse || networkResponse.status !== 200 || networkResponse.type !== 'basic') {
                            return networkResponse;
                        }

                        // Les requêtes d'API (comme api/weather.js) ne doivent pas être mises en cache
                        if (event.request.url.includes('/api/')) {
                            return networkResponse;
                        }

                        // Cloner la réponse pour la mettre en cache et la retourner au navigateur
                        const responseToCache = networkResponse.clone();

                        caches.open(CACHE_NAME)
                            .then(cache => {
                                cache.put(event.request, responseToCache);
                            });

                        return networkResponse;
                    }
                );
            })
    );
});

// --- 3. Gestion de la Mise à Jour (Nettoyage des Anciens Caches) ---
self.addEventListener('activate', event => {
    const cacheWhitelist = [CACHE_NAME];

    event.waitUntil(
        caches.keys().then(cacheNames => {
            return Promise.all(
                cacheNames.map(cacheName => {
                    if (cacheWhitelist.indexOf(cacheName) === -1) {
                        console.log('Service Worker: Suppression de l\'ancien cache:', cacheName);
                        return caches.delete(cacheName);
                    }
                })
            );
        })
    );
    // Prend le contrôle des clients sans avoir besoin d'actualiser la page.
    self.clients.claim();
});

// --- 4. Gestion des Requêtes Push (Si le manifest le supporte) ---
// (Optionnel : Peut être étendu plus tard si vous ajoutez des notifications push)
/*
self.addEventListener('push', event => {
    console.log('[Service Worker] Push Reçu.');
    // ... code pour afficher la notification ...
});
*/
