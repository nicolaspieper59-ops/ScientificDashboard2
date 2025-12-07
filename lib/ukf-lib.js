// =================================================================
// FICHIER : lib/ukf-lib.js
// VERSION : FINALE CORRIGÉE (Compatible gnss-dashboard-full 15.4)
// DÉPENDANCE : math.js (https://cdnjs.cloudflare.com/ajax/libs/mathjs/9.4.4/math.min.js)
// =================================================================

(function(window) {
    /**
     * Classe ProfessionalUKF (Unscented Kalman Filter)
     * Architecture à 21 États simplifiée pour la fusion GNSS/IMU.
     */
    class ProfessionalUKF {
        constructor() {
            // 1. Vérification critique des dépendances
            if (typeof math === 'undefined') {
                console.error("🔴 ERREUR CRITIQUE UKF : La librairie math.js n'est pas chargée.");
                throw new Error("math.js est requis pour le UKF.");
            }

            console.log("UKF 21 États : Initialisation...");

            // 2. Définition des dimensions
            this.N_STATES = 21; 

            // 3. Initialisation du Vecteur d'État (x)
            // Index: 0=Lat, 1=Lon, 2=Alt, 3=vN, 4=vE, 5=vD, 6-9=Quaternion, 10-20=Biais/Erreurs
            this.x = math.zeros(this.N_STATES); 

            // 4. Initialisation de la Matrice de Covariance (P)
            // Diagonale avec une incertitude initiale modérée
            this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 

            // 5. Matrice de Bruit de Processus (Q)
            // Faible bruit pour permettre au modèle d'évoluer doucement
            this.Q = math.diag(Array(this.N_STATES).fill(1e-6));

            // Constantes géodésiques
            this.R_E = 6371000; // Rayon terrestre moyen
            this.D2R = Math.PI / 180;
            this.R2D = 180 / Math.PI;

            console.log("UKF 21 États : Prêt. 🟢");
        }

        /**
         * Étape de Prédiction (Propagation IMU)
         * @param {object} imuData - { accel: [ax, ay, az], gyro: [gx, gy, gz] }
         * @param {number} dt - Delta temps en secondes
         */
        predict(imuData, dt) {
            if (dt <= 0) return;

            // 1. Récupération de l'état actuel
            let lat = this.x.get([0]);
            let vn = this.x.get([3]);
            
            // 2. Traitement simplifié de l'accélération (Correction de biais basique)
            // Dans une implémentation complète, on utiliserait les quaternions ici.
            const accel_bias_x = this.x.get([9]) || 0;
            const ax_corrected = imuData.accel[0] - accel_bias_x;

            // 3. Propagation de la vitesse (V = V0 + a*t)
            let vn_pred = vn + ax_corrected * dt;
            
            // ZUPT (Zero Velocity Update) simple : si vitesse très faible, on annule
            if (Math.abs(vn_pred) < 0.01) vn_pred = 0;

            // 4. Propagation de la position (Lat = Lat0 + (Vn / R) * dt)
            // Conversion en radians pour le calcul
            let lat_rad = lat * this.D2R; 
            let lat_pred_rad = lat_rad + (vn_pred / this.R_E) * dt;
            let lat_pred_deg = lat_pred_rad * this.R2D;

            // 5. Mise à jour du vecteur d'état prédit
            this.x.set([0], lat_pred_deg); // Latitude
            this.x.set([3], vn_pred);      // Vitesse Nord/Longitudinale

            // 6. Propagation de la covariance (P = F*P*F' + Q)
            // Simplification : On ajoute simplement le bruit de processus
            this.P = math.add(this.P, this.Q);
        }

        /**
         * Étape de Mise à Jour (Correction GNSS)
         * @param {object} gpsData - { latitude, longitude, altitude, speed, accuracy }
         * @param {number} R_dyn - Bruit de mesure dynamique calculé par l'environnement
         */
        update(gpsData, R_dyn) {
            if (!gpsData) return;

            // Gain de Kalman (K) simplifié pour la stabilité du modèle
            // Dans une version complète, K = P * H' * inv(H * P * H' + R)
            // Ici, nous utilisons un facteur de pondération adaptatif basé sur R_dyn
            const confidence = Math.max(0.01, Math.min(0.5, 100 / (R_dyn + 1))); 
            const K = confidence; 

            // 1. Correction Latitude
            const lat_mes = gpsData.latitude;
            const lat_est = this.x.get([0]);
            if (lat_est === 0) {
                this.x.set([0], lat_mes); // Initialisation
            } else {
                this.x.set([0], lat_est + K * (lat_mes - lat_est));
            }

            // 2. Correction Longitude
            const lon_mes = gpsData.longitude;
            const lon_est = this.x.get([1]);
            if (lon_est === 0) {
                this.x.set([1], lon_mes);
            } else {
                this.x.set([1], lon_est + K * (lon_mes - lon_est));
            }

            // 3. Correction Altitude
            const alt_mes = gpsData.altitude || 0;
            const alt_est = this.x.get([2]);
            this.x.set([2], alt_est + K * (alt_mes - alt_est));

            // 4. Correction Vitesse (si disponible via GPS Doppler)
            if (gpsData.speed !== null && gpsData.speed !== undefined) {
                const spd_mes = gpsData.speed;
                const spd_est = this.x.get([3]);
                this.x.set([3], spd_est + K * (spd_mes - spd_est));
            }

            // 5. Réduction de la Covariance (Incertitude)
            // P = (I - K*H) * P
            // Simplification : on réduit la diagonale
            const reductionFactor = 1 - (K * 0.5);
            this.P = math.multiply(this.P, reductionFactor);
        }

        /**
         * Extrait l'état actuel pour l'affichage dans le Dashboard
         */
        getState() {
            const x_data = this.x.toArray(); // Conversion vecteur math.js -> Array JS
            
            // Calcul de la vitesse scalaire 3D
            const vN = x_data[3];
            const vE = x_data[4];
            const vD = x_data[5];
            const speed3D = Math.sqrt(vN**2 + vE**2 + vD**2);

            return {
                lat: x_data[0], // Déjà en degrés
                lon: x_data[1],
                alt: x_data[2],
                vN: vN,
                vE: vE,
                vD: vD,
                speed: speed3D,
                // Extraction de l'incertitude (diagonale de P) pour l'affichage
                kUncert: this.P.get([3, 3]) + this.P.get([4, 4]) + this.P.get([5, 5]),
                kAltUncert: this.P.get([2, 2])
            };
        }
    }

    // Exposition globale
    window.ProfessionalUKF = ProfessionalUKF;

})(window);
