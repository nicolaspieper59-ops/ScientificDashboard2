self.addEventListener('install', event => {
  event.waitUntil(
    caches.open('dashboard-cache-v1').then(cache => cache.addAll([
      '/',
      '/index.html',
      '/cockpit.js',
      '/manifest.json',
      '/styles.css',
      // Ajoutez ici d'autres fichiers à mettre en cache
    ]))
  );
  self.skipWaiting();
});

self.addEventListener('fetch', event => {
  event.respondWith(
    caches.match(event.request).then(response => response || fetch(event.request))
  );
}); 
