// =================================================================
// FICHIER : lib/ukf-lib.js
// VERSION : FINALE ULTIME CONSOLIDÉE - UKF 21 États / 43 Sigma Points
// DÉPENDANCE CRITIQUE : math.js (https://cdnjs.cloudflare.com/ajax/libs/mathjs/x.x.x/math.min.js)
// =================================================================

(function(window) {
    
    // --- CONSTANTES CRITIQUES (WGS84 & UKF) ---
    const D2R = Math.PI / 180;
    const R2D = 180 / Math.PI;
    const N_STATES = 21; 
    const N_SIGMA = 2 * N_STATES + 1; // 43 Sigma Points (2*21 + 1)
    const GRAVITY_STD = 9.80665; // Gravité standard (m/s²)
    
    // Paramètres WGS84 (Ellipsoïde Terre)
    const WGS84_A = 6378137.0;  
    const WGS84_F = 1 / 298.257223563; 
    const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; 

    /**
     * Classe ProfessionalUKF (Unscented Kalman Filter)
     * Architecture à 21 États : 
     * [0-2: Pos (Lat, Lon, Alt); 3-5: Vel (vN, vE, vD); 6-9: Quat (qX, qY, qZ, qW); 
     * 10-12: Biais Acc X-Z; 13-15: Biais Gyro X-Z; 16: Biais Baro; 17-20: Réservé/Erreurs]
     */
    class ProfessionalUKF {
        constructor() {
            if (typeof math === 'undefined') {
                console.error("🔴 ERREUR CRITIQUE UKF : La librairie math.js n'est pas chargée.");
                throw new Error("math.js est requis pour le UKF.");
            }

            console.log("UKF 21 États : Initialisation...");

            // --- 1. CONFIGURATION UKF (Paramètres Scalaires) ---
            this.alpha = 1e-3; 
            this.beta = 2;     
            this.kappa = 0;    
            this.lambda = (this.alpha * this.alpha * N_STATES) - N_STATES;
            
            // Poids (W) des Sigma Points
            this.Wm = math.zeros(N_SIGMA);
            this.Wc = math.zeros(N_SIGMA);
            this.Wm.set([0], this.lambda / (N_STATES + this.lambda));
            this.Wc.set([0], this.Wm.get([0]) + (1 - this.alpha * this.alpha + this.beta));
            for (let i = 1; i < N_SIGMA; i++) {
                const W = 1 / (2 * (N_STATES + this.lambda));
                this.Wm.set([i], W);
                this.Wc.set([i], W);
            }
            
            // --- 2. VECTEUR D'ÉTAT (x) et COVARIANCE (P) ---
            this.x = math.zeros(N_STATES); 
            this.x.set([9], 1.0); // qW = 1 (Quaternion d'identité)

            const P_initial_diag = Array(N_STATES).fill(1e-2).map((val, i) => {
                if (i < 3) return 10.0;  // Pos
                if (i < 6) return 1.0;   // Vel
                if (i < 10) return 0.01; // Quat
                if (i >= 10 && i <= 15) return 1e-4; // Biais IMU
                return 0.001;            
            });
            this.P = math.diag(P_initial_diag);

            // --- 3. BRUIT DE PROCESSUS (Q) ---
            this.Q = this._getProcessNoiseMatrix('NORMAL'); 
            this.lastUpdateTime = Date.now() / 1000;
        }
        
        // =========================================================
        // A. UTILITAIRES UKF & QUATERNION (Pour la clarté)
        // =========================================================
        
        _getProcessNoiseMatrix(mode) {
            let q_pos = 1e-7, q_vel = 1e-4, q_att = 1e-5, q_bias = 1e-8;
            if (mode === 'ROBUST') { q_pos = 1e-5; q_vel = 1e-2; q_att = 1e-4; } 
            else if (mode === 'MAX_PRECISION') { q_pos = 1e-9; q_vel = 1e-6; q_att = 1e-6; }
            
            const Q_diag = Array(N_STATES).fill(0).map((_, i) => {
                if (i < 3) return q_pos;  
                if (i < 6) return q_vel;  
                if (i < 10) return q_att; 
                return q_bias;           
            });
            return math.diag(Q_diag);
        }

        _normalizeQuat(q) {
            const mag = Math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2);
            return mag > 1e-6 ? q.map(v => v / mag) : [0, 0, 0, 1];
        }

        _quatMultiply(q1, q2) {
            const [x1, y1, z1, w1] = q1;
            const [x2, y2, z2, w2] = q2;
            return [
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            ];
        }
        
        _quatToDCM(q) {
            const [qx, qy, qz, qw] = q;
            const qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz, qw2 = qw * qw;
            const dcm = [
                [qw2 + qx2 - qy2 - qz2, 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                [2 * (qx * qy + qw * qz), qw2 - qx2 + qy2 - qz2, 2 * (qy * qz - qw * qx)],
                [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), qw2 - qx2 - qy2 + qz2]
            ];
            return math.matrix(dcm);
        }
        
        _updateQuat(q_current, omega_corr, dt) {
            const [w_x, w_y, w_z] = omega_corr;
            const q_delta_vec = [w_x * dt * 0.5, w_y * dt * 0.5, w_z * dt * 0.5, 1.0];
            const q_delta = this._normalizeQuat(q_delta_vec);
            const q_new = this._quatMultiply(q_current, q_delta);
            return this._normalizeQuat(q_new);
        }

        /**
         * Génère les 43 Sigma Points (2N+1)
         * @returns {Array<math.Matrix>} La matrice des Sigma Points [N_STATES x N_SIGMA]
         */
        _generateSigmaPoints(x_center = this.x, P_cov = this.P) {
            // ⚠️ LOGIQUE UKF COMPLÈTE : Factorisation de Cholesky
            let L;
            try {
                L = math.cholesky(math.multiply((N_STATES + this.lambda), P_cov));
            } catch (e) {
                // Tentative de récupération en rendant P définie positive
                console.warn("UKF: Matrice P non définie positive. Récupération...");
                P_cov = math.add(P_cov, math.diag(Array(N_STATES).fill(1e-6)));
                L = math.cholesky(math.multiply((N_STATES + this.lambda), P_cov));
            }

            const SigmaPoints = [];
            // Point 0 : Le point central
            SigmaPoints.push(x_center.clone()); 

            for (let i = 0; i < N_STATES; i++) {
                const Li = math.column(L, i); 
                // Points i + 1 et i + N_STATES + 1
                SigmaPoints.push(math.add(x_center, Li)); 
                SigmaPoints.push(math.subtract(x_center, Li)); 
            }
            return SigmaPoints; 
        }

        // =========================================================
        // B. ÉTAPE DE PRÉDICTION (PROPAGATION DES SIGMA POINTS)
        // =========================================================

        predict(accX, accY, accZ, gyroX = 0, gyroY = 0, gyroZ = 0) {
            const dt = (Date.now() / 1000) - this.lastUpdateTime;
            if (dt <= 0) return;

            // --- 1. GÉNÉRATION DES SIGMA POINTS ---
            const X_sigma = this._generateSigmaPoints();
            const X_pred = []; 
            
            // --- 2. PROPAGATION NON-LINÉAIRE (f(x)) ---
            for (let i = 0; i < N_SIGMA; i++) {
                let x_current = X_sigma[i];

                // 2.1. Correction IMU avec le Biais du point sigma (États 10-15)
                const acc_corr = [accX - x_current.get([10]), accY - x_current.get([11]), accZ - x_current.get([12])];
                const gyro_corr = [(gyroX - x_current.get([13])) * D2R, (gyroY - x_current.get([14])) * D2R, (gyroZ - x_current.get([15])) * D2R];

                // 2.2. Mise à jour du Quaternion (États 6 à 9)
                const q_current = [x_current.get([6]), x_current.get([7]), x_current.get([8]), x_current.get([9])];
                const q_new = this._updateQuat(q_current, gyro_corr, dt);
                
                // Mettre à jour le quaternion du point sigma (très important)
                x_current.set([6], q_new[0]); x_current.set([7], q_new[1]);
                x_current.set([8], q_new[2]); x_current.set([9], q_new[3]);

                // 2.3. Transformation de l'Accélération (Body -> NED)
                const DCM = this._quatToDCM(q_new);
                const acc_NED = math.multiply(DCM, math.matrix([acc_corr[0], acc_corr[1], acc_corr[2]])); 
                
                // 2.4. Mise à jour de la Vitesse (États 3 à 5)
                const vn = x_current.get([3]);
                const ve = x_current.get([4]);
                const vd = x_current.get([5]);
                
                const acc_N_total = acc_NED.get([0]);
                const acc_E_total = acc_NED.get([1]);
                const acc_D_total = acc_NED.get([2]) + GRAVITY_STD; // Inclure la gravité
                
                x_current.set([3], vn + acc_N_total * dt); // vN
                x_current.set([4], ve + acc_E_total * dt); // vE
                x_current.set([5], vd + acc_D_total * dt); // vD
                
                // 2.5. Mise à jour de la Position (États 0 à 2 - LLA)
                const lat = x_current.get([0]) * D2R; 
                const alt = x_current.get([2]);
                const Rm = WGS84_A * (1 - WGS84_E2) / Math.pow(1 - WGS84_E2 * Math.sin(lat) * Math.sin(lat), 1.5);
                const Rn = WGS84_A / Math.sqrt(1 - WGS84_E2 * Math.sin(lat) * Math.sin(lat));

                const d_lat = (x_current.get([3]) / (Rm + alt)) * dt;
                const d_lon = (x_current.get([4]) / ((Rn + alt) * Math.cos(lat))) * dt;
                const d_alt = -x_current.get([5]) * dt; 

                x_current.set([0], x_current.get([0]) + d_lat * R2D); 
                x_current.set([1], x_current.get([1]) + d_lon * R2D); 
                x_current.set([2], alt + d_alt);       
                
                // 2.6. Propagation des Biais (États 10-16)
                // Le biais est traité comme une marche aléatoire (constante + bruit Q déjà inclus)
                // Donc, aucune modification n'est nécessaire ici pour un modèle de biais constant.
                
                X_pred.push(x_current);
            }
            
            // --- 3. RECONSTRUCTION DE L'ÉTAT (x_pred) ET COVARIANCE (P_pred) ---

            // 3.1. Reconstruction de l'état central (Moyenne pondérée)
            let x_pred = math.zeros(N_STATES);
            for (let i = 0; i < N_SIGMA; i++) {
                x_pred = math.add(x_pred, math.multiply(this.Wm.get([i]), X_pred[i]));
            }
            
            // 3.2. Reconstruction de la Covariance (P_pred = sum(Wc * dX * dX.T) + Q)
            let P_pred = this.Q.clone(); 
            for (let i = 0; i < N_SIGMA; i++) {
                const dX = math.subtract(X_pred[i], x_pred);
                P_pred = math.add(P_pred, math.multiply(this.Wc.get([i]), math.multiply(dX, math.transpose(dX))));
            }
            
            this.x = x_pred; 
            this.P = P_pred; 
            
            // Post-Correction : Renormaliser l'état Quaternion de l'état central reconstruit
            const q_final = [this.x.get([6]), this.x.get([7]), this.x.get([8]), this.x.get([9])];
            const q_normalized = this._normalizeQuat(q_final);
            this.x.set([6], q_normalized[0]); this.x.set([7], q_normalized[1]);
            this.x.set([8], q_normalized[2]); this.x.set([9], q_normalized[3]);
            
            this.lastUpdateTime = Date.now() / 1000;
        }

        // =========================================================
        // C. ÉTAPE DE MISE À JOUR (CORRECTION)
        // =========================================================

        update(measurement, sourceTag) {
            if (!measurement) return;
            
            // --- 1. DÉFINITION DE R (Mesure) ---
            let R_base_val = 10.0; 
            let R_dim = 3; 
            if (sourceTag === 'RTK_VIO_FUSION') { R_base_val = 0.005; } 
            else if (sourceTag === 'VIO') { R_base_val = 0.05; }
            else if (sourceTag === 'GNSS') { R_base_val = measurement.acc || 1.5; } 
            else if (sourceTag === 'BARO' || sourceTag === 'UWB_RANGING') { R_base_val = 0.5; R_dim = 1; } 
            const R_diag = Array(R_dim).fill(R_base_val * R_base_val);
            const R = math.diag(R_diag);
            
            // --- 2. TRANSFORMATION DES SIGMA POINTS (h(x)) ---
            const X_pred = this._generateSigmaPoints(); 
            const Z_pred = []; 

            // Fonction h(x) : Mappe l'état UKF (x) à l'espace de mesure (z)
            for (let i = 0; i < N_SIGMA; i++) {
                let z_vector;
                const alt_pred = X_pred[i].get([2]); // Altitude
                const baro_bias_pred = X_pred[i].get([16]); // Biais Baro

                if (sourceTag === 'BARO') {
                    // Mesure Z = [Alt + Biais Baro]
                    z_vector = [alt_pred + baro_bias_pred];
                } else {
                    // Mesure Z = [Lat, Lon, Alt] ou [Distance UWB] (si UWB)
                    z_vector = [X_pred[i].get([0]), X_pred[i].get([1]), alt_pred];
                }
                Z_pred.push(math.matrix(z_vector).resize([R_dim]));
            }
            
            // Reconstruction de la mesure moyenne prédite (z_pred_moy)
            let z_pred_moy = math.zeros(R_dim);
            for (let i = 0; i < N_SIGMA; i++) {
                z_pred_moy = math.add(z_pred_moy, math.multiply(this.Wm.get([i]), Z_pred[i]));
            }
            
            // --- 3. CALCUL DE L'INNOVATION, S ET Pxz ---
            let Z_mes;
            if (R_dim === 3) { Z_mes = math.matrix([measurement.lat, measurement.lon, measurement.alt]); }
            else if (R_dim === 1) { Z_mes = math.matrix([measurement.alt || 0]); } // Baro (Alt)
            
            const y = math.subtract(Z_mes, z_pred_moy); // Innovation y = z_mes - z_pred_moy
            
            let S = R.clone(); // Covariance d'innovation
            let Pxz = math.zeros(N_STATES, R_dim); // Cross-Covariance
            
            for (let i = 0; i < N_SIGMA; i++) {
                const dX = math.subtract(X_pred[i], this.x); 
                const dZ = math.subtract(Z_pred[i], z_pred_moy);

                S = math.add(S, math.multiply(this.Wc.get([i]), math.multiply(dZ, math.transpose(dZ))));
                Pxz = math.add(Pxz, math.multiply(this.Wc.get([i]), math.multiply(dX, math.transpose(dZ))));
            }

            // --- 4. CALCUL DU GAIN DE KALMAN (K) et CORRECTION ---
            let K;
            try {
                K = math.multiply(Pxz, math.inv(S));
            } catch (e) {
                console.error("UKF: Échec de l'inversion de S. Correction annulée.");
                return; 
            }
            
            // Correction de l'état : x = x + K * y
            this.x = math.add(this.x, math.multiply(K, y));

            // Correction de la Covariance : P = P - K * S * K.T
            this.P = math.subtract(this.P, math.multiply(math.multiply(K, S), math.transpose(K)));
            
            // Post-Correction : Renormalisation du Quaternion
            const q_final = [this.x.get([6]), this.x.get([7]), this.x.get([8]), this.x.get([9])];
            const q_normalized = this._normalizeQuat(q_final);
            this.x.set([6], q_normalized[0]); this.x.set([7], q_normalized[1]);
            this.x.set([8], q_normalized[2]); this.x.set([9], q_normalized[3]);
        }

        // =========================================================
        // D. ACCESSEURS D'ÉTAT
        // =========================================================

        getState() {
            const x_data = this.x.toArray(); 
            const kUncert = (this.P.get([0, 0]) + this.P.get([1, 1]) + this.P.get([2, 2])) / 3;

            return {
                lat: x_data[0], lon: x_data[1], alt: x_data[2],
                vN: x_data[3], vE: x_data[4], vD: x_data[5],
                speed: Math.sqrt(x_data[3]**2 + x_data[4]**2 + x_data[5]**2),
                kUncert: Math.sqrt(kUncert), 
                baroBias: x_data[16],
                qX: x_data[6], qY: x_data[7], qZ: x_data[8], qW: x_data[9],
                accBiasX: x_data[10], gyroBiasX: x_data[13] 
            };
        }
    }

    // Exposez la classe UKF au contexte global
    window.ProfessionalUKF = ProfessionalUKF;

})(window);
