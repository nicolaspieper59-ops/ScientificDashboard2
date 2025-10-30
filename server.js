// ==========================================================
// FICHIER : server.js (Serveur Proxy Node.js pour OpenWeatherMap)
// ----------------------------------------------------------
// Ce fichier doit être placé à la racine de votre dépôt ScientificDashboard2.
// ==========================================================
const express = require('express');
const fetch = require('node-fetch'); // Nécessite l'installation via npm

const app = express();
// L'hébergeur (Vercel ou Render) fournira le port via process.env.PORT
const PORT = process.env.PORT || 3000; 

// 1. Récupération de la clé API
// La clé est stockée SECRÈTEMENT sur Vercel/Render sous le nom OPENWEATHER_API_KEY
const API_KEY = process.env.OPENWEATHER_API_KEY; 

// 2. Configuration des CORS
// Autorise l'accès à l'API depuis n'importe quel domaine (pour simplifier le développement)
app.use((req, res, next) => {
    res.setHeader('Access-Control-Allow-Origin', '*'); // À ajuster si vous avez une URL front-end fixe
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
    next();
});


// 3. Route de l'API Proxy
// URL d'appel : /api/weather?lat=XX.XX&lon=YY.YY
app.get('/api/weather', async (req, res) => {
    // Vérification de la clé API (sécurité)
    if (!API_KEY) {
        return res.status(500).json({ error: 'Clé API OpenWeatherMap non configurée sur le serveur.' });
    }

    const { lat, lon } = req.query;

    if (!lat || !lon) {
        return res.status(400).json({ error: 'Les paramètres lat et lon sont requis.' });
    }

    // Construction de l'URL OpenWeatherMap (en utilisant le système métrique)
    const OWM_URL = `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${API_KEY}&units=metric`;

    try {
        const response = await fetch(OWM_URL);
        const data = await response.json();

        // Si l'API OpenWeatherMap retourne une erreur, la propager
        if (response.status !== 200) {
            console.error(`Erreur OWM: ${data.message}`);
            return res.status(response.status).json({ error: data.message });
        }

        // Retourner les données météo au dashboard front-end
        res.status(200).json(data);

    } catch (error) {
        console.error("Erreur lors de la récupération des données OWM:", error);
        res.status(500).json({ error: 'Erreur interne du serveur proxy.' });
    }
});


// 4. Message de Bienvenue (Route Racine)
app.get('/', (req, res) => {
    res.status(200).send('Serveur Proxy GNSS Météo en cours d\'exécution. Utilisez /api/weather?lat=...&lon=...');
});


// 5. Démarrage du Serveur
app.listen(PORT, () => {
    console.log(`🚀 Serveur proxy démarré sur le port ${PORT}`);
    console.log(`URL de test (local) : http://localhost:${PORT}/api/weather?lat=48.86&lon=2.35`);
});
