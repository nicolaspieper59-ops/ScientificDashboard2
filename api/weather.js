// =================================================================
// FICHIER PROXY SERVERLESS : api/weather.js
// Déployable sur Vercel (accessible via VOTRE-URL/api/weather)
// =================================================================

const fetch = require('node-fetch'); // Utilisé pour effectuer la requête HTTP sortante

// 🚨 Configuration Critique 🚨
// La clé API DOIT être stockée dans une variable d'environnement sur Vercel,
// nommée par exemple OPENWEATHER_API_KEY.
const OPENWEATHER_API_KEY = process.env.OPENWEATHER_API_KEY || 'CLE_API_MANQUANTE';
const OPENWEATHER_BASE_URL = 'https://api.openweathermap.org/data/2.5/weather';

module.exports = async (req, res) => {
    // 1. Gestion des en-têtes CORS (Crucial pour que le navigateur accepte la réponse)
    res.setHeader('Access-Control-Allow-Origin', '*'); // Autorise tous les domaines (votre dashboard)
    res.setHeader('Access-Control-Allow-Methods', 'GET, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'X-Requested-With, Content-Type, Accept');

    // Gérer la requête OPTIONS (pré-vol CORS)
    if (req.method === 'OPTIONS') {
        res.status(200).end();
        return;
    }

    // 2. Vérification de la clé API
    if (OPENWEATHER_API_KEY === 'CLE_API_MANQUANTE' || !OPENWEATHER_API_KEY) {
        res.status(500).json({ error: 'OPENWEATHER_API_KEY non configurée. Veuillez l\'ajouter aux variables d\'environnement de Vercel.' });
        return;
    }

    // 3. Récupération des paramètres (latitude et longitude)
    const { lat, lon } = req.query;

    if (!lat || !lon) {
        res.status(400).json({ error: 'Paramètres lat et lon manquants dans l\'URL.' });
        return;
    }

    try {
        // 4. Construction de l'URL OpenWeatherMap (units=metric pour Celsius)
        const weatherUrl = `${OPENWEATHER_BASE_URL}?lat=${lat}&lon=${lon}&appid=${OPENWEATHER_API_KEY}&units=metric&lang=fr`;

        // 5. Appel à l'API externe
        const response = await fetch(weatherUrl);
        const data = await response.json();

        if (!response.ok) {
            // Renvoyer l'erreur de l'API OpenWeatherMap
            console.error('Erreur API Météo:', data);
            res.status(response.status).json({ error: data.message || 'Erreur lors de la récupération des données météo.' });
            return;
        }

        // 6. Succès : Retourner les données reçues directement au client JS
        res.status(200).json(data);

    } catch (error) {
        console.error('Erreur de serveur interne du proxy:', error);
        res.status(500).json({ error: 'Erreur interne du serveur proxy.' });
    }
};
