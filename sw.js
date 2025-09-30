self.addEventListener('install', event => {
  event.waitUntil(
    caches.open('vitesse-cache').then(cache => {
      return cache.addAll([
        './',
        './index.html',
        './style.css',
        './manifest.json',
        './js/compteurVitesse.js'
      ]);
    })
  );
});

self.addEventListener('fetch', event => {
  event.respondWith(
    caches.match(event.request).then(response => response || fetch(event.request))
  );
});
