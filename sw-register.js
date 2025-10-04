<script>
if ('serviceWorker' in navigator) {
  navigator.serviceWorker.register('/sw.js')
    .then(reg => console.log('SW enregistré avec succès:', reg))
    .catch(err => console.error('Échec enregistrement SW:', err));
}
</script>
