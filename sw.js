const CACHE_NAME = 'scientific-dashboard-v1';
const urlsToCache = [
  './',
  './index.html',
  './sw-register.js'
];

// Installer le service worker et mettre en cache les fichiers
self.addEventListener('install', event => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(cache => cache.addAll(urlsToCache))
      .then(() => self.skipWaiting())
  );
});

// Activer le service worker et nettoyer les anciennes caches
self.addEventListener('activate', event => {
  event.waitUntil(
    caches.keys().then(keys => 
      Promise.all(
        keys.filter(key => key !== CACHE_NAME)
            .map(key => caches.delete(key))
      )
    )
  );
  self.clients.claim();
});

// Intercepter les requ¨ºtes et renvoyer depuis le cache si hors-ligne
self.addEventListener('fetch', event => {
  event.respondWith(
    caches.match(event.request)
      .then(response => response || fetch(event.request).catch(() => 
        new Response('Ressource non disponible hors-ligne.', {status:404})
      ))
  );
});
