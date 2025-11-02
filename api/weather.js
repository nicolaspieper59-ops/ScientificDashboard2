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
        // 4. Construction de l'URL finale (avec unités métriques et langue française)
        const finalUrl = `${OPENWEATHER_BASE_URL}?lat=${lat}&lon=${lon}&units=metric&lang=fr&appid=${OPENWEATHER_API_KEY}`;

        // 5. Appel à l'API OpenWeatherMap
        const apiResponse = await fetch(finalUrl);
        const data = await apiResponse.json();

        if (!apiResponse.ok) {
            // Gérer les erreurs de l'API externe (ex: clé API invalide, ville non trouvée)
            const errorMsg = data.message || `Erreur de l'API externe avec code ${apiResponse.status}`;
            res.status(apiResponse.status).json({ error: errorMsg });
            return;
        }

        // 6. Succès : renvoyer les données au client (votre tableau de bord)
        res.status(200).json(data);

    } catch (error) {
        console.error("Erreur serveur lors de l'appel à OpenWeatherMap:", error);
        res.status(500).json({ error: 'Erreur interne du serveur proxy.', details: error.message });
    }
};
