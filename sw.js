// =================================================================
// FICHIER SW.JS (SERVICE WORKER)
// =================================================================

const CACHE_NAME = 'ekf-ins-cache-v1.0.3'; 
const OFFLINE_URLS = [
    // Fichiers statiques et librairies locales
    'index.html',
    'style.css', 
    'manifest.json',
    'fontawesome.min.css', 
    'math.min.js',
    'leaflet.js',
    'leaflet.css',
    'suncalc.js', 
    'gnss-dashboard-full.js',
    'weather.js', 
    'sw.js'
    // Ajouter les icônes ici
];

// --- 1. ÉVÉNEMENT INSTALL ---
self.addEventListener('install', (event) => {
    event.waitUntil(
        caches.open(CACHE_NAME)
            .then((cache) => {
                return cache.addAll(OFFLINE_URLS);
            })
    );
    self.skipWaiting(); 
});

// --- 2. ÉVÉNEMENT ACTIVATE ---
self.addEventListener('activate', (event) => {
    event.waitUntil(
        caches.keys().then((cacheNames) => {
            return Promise.all(
                cacheNames.map((cacheName) => {
                    if (cacheName !== CACHE_NAME) {
                        return caches.delete(cacheName);
                    }
                })
            );
        })
    );
    return self.clients.claim(); 
});

// --- 3. ÉVÉNEMENT FETCH (Cache-First) ---
self.addEventListener('fetch', (event) => {
    event.respondWith(
        caches.match(event.request)
            .then((response) => {
                if (response) {
                    return response;
                }
                return fetch(event.request)
                    .catch(() => {
                        // Retourne une réponse en cas d'échec réseau
                    });
            })
    );
});
