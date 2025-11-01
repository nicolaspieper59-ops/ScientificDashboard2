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
    );
    self.skipWaiting(); // Force l'activation immédiate du nouveau SW
});

// Étape 2: Activation du Service Worker et nettoyage des anciens caches
self.addEventListener('activate', event => {
    console.log('[Service Worker] Activation et nettoyage des anciens caches...');
    event.waitUntil(
        caches.keys().then(cacheNames => {
            return Promise.all(
                cacheNames.map(cacheName => {
                    if (cacheName !== CACHE_NAME) {
                        console.log('[Service Worker] Suppression de l’ancien cache:', cacheName);
                        return caches.delete(cacheName);
                    }
                })
            );
        })
    );
});

// Étape 3: Stratégie Cache-First pour les ressources
self.addEventListener('fetch', event => {
    // Laissons les requêtes externes dynamiques (API météo, NTP) passer au réseau
    if (event.request.url.includes(self.location.origin) && event.request.method === 'GET') {
        event.respondWith(
            caches.match(event.request)
                .then(response => {
                    // Cache Hit - Servir la ressource depuis le cache
                    if (response) {
                        return response;
                    }
                    
                    // Cache Miss - Essayer de récupérer la ressource via le réseau et l'ajouter au cache
                    return fetch(event.request).then(
                        response => {
                            // Vérifie si la réponse est valide
                            if(!response || response.status !== 200 || response.type !== 'basic') {
                                return response;
                            }
                            
                            // Clone la réponse pour la mettre dans le cache et la retourner
                            const responseToCache = response.clone();
                            caches.open(CACHE_NAME)
                                .then(cache => {
                                    cache.put(event.request, responseToCache);
                                });

                            return response;
                        }
                    ).catch(error => {
                        // Ceci est l'erreur si vous êtes hors ligne et que le fichier n'est pas dans le cache
                        console.error('[Service Worker] Fetch failed; hors ligne:', event.request.url, error);
                        // Vous pouvez retourner une page "hors ligne" ici si vous en avez une
                        return new Response("Hors ligne : ressource non disponible dans le cache.");
                    });
                })
        );
    }
});
