if ('serviceWorker' in navigator) {
  navigator.serviceWorker.register('./sw.js').then(() => {
    console.log("🛰️ Mode hors ligne activé");
  });
}
