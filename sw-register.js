if ('serviceWorker' in navigator) {
  window.addEventListener('load', () => {
    navigator.serviceWorker.register('sw.js')
      .then(reg => console.log('✅ Service Worker enregistré', reg))
      .catch(err => console.error('❌ Erreur SW:', err));
  });
}
