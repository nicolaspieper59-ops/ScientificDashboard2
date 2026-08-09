importScripts('./vsop2013.js', './ElpMpp02DE_min.js');
const DELTA_T_SEC = 69.18;

self.onmessage = function(e) {
    const { timestampUtc, station3D, astres } = e.data;
    if (!timestampUtc || !station3D.gpsAcquis) return;

    try {
        const jd_UTC = (timestampUtc / 86400000.0) + 2440587.5;
        const jd_TT  = jd_UTC + (DELTA_T_SEC / 86400.0);
        const T      = (jd_TT - 2451545.0) / 36525.0;

        const dUT1 = jd_UTC - 2451545.0;
        const eraRad = (2.0 * Math.PI) * (0.7790572732640 + 1.00273781191135448 * dUT1);

        const latRad = station3D.lat * Math.PI / 180.0;
        const lonRad = station3D.lon * Math.PI / 180.0;
        const cosLat = Math.cos(latRad), sinLat = Math.sin(latRad);
        const cosLon = Math.cos(lonRad), sinLon = Math.sin(lonRad);

        let pT = vsop2013.Earth.position(T);
        const terreHelio = { x: pT.x, y: pT.y, z: pT.z };
        let resultats = {};

        astres.forEach(astre => {
            let vecJ2000UA = { x: 0, y: 0, z: 0 };
            if (astre === 'soleil') {
                vecJ2000UA = { x: -terreHelio.x, y: -terreHelio.y, z: -terreHelio.z };
            } else if (astre === 'lune') {
                let pLune = getX2000_DE(T);
                vecJ2000UA = { x: pLune.X, y: pLune.Y, z: pLune.Z };
            } else if (vsop2013[astre] && typeof vsop2013[astre].position === 'function') {
                let pA = vsop2013[astre].position(T);
                vecJ2000UA = { x: pA.x - terreHelio.x, y: pA.y - terreHelio.y, z: pA.z - terreHelio.z };
            } else { return; }

            const UA_TO_M = 149597870700.0;
            const xM = vecJ2000UA.x * UA_TO_M, yM = vecJ2000UA.y * UA_TO_M, zM = vecJ2000UA.z * UA_TO_M;

            const xECEF =  xM * Math.cos(eraRad) + yM * Math.sin(eraRad);
            const yECEF = -xM * Math.sin(eraRad) + yM * Math.cos(eraRad);
            const zECEF =  zM;

            const dx = xECEF - station3D.X_itrs, dy = yECEF - station3D.Y_itrs, dz = zECEF - station3D.Z_itrs;
            const east  = -sinLon * dx + cosLon * dy;
            const north = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
            const up    =  cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;

            const distM = Math.sqrt(east * east + north * north + up * up);
            let elRad = Math.asin(up / distM), azRad = Math.atan2(east, north);
            if (azRad < 0) azRad += 2.0 * Math.PI;

            let elDeg = elRad * 180.0 / Math.PI, azDeg = azRad * 180.0 / Math.PI;

            // Billard météo 10 strates
            let elRefractee = elDeg, inflexionStr = "0.0000°";
            if (elDeg > 0 && elDeg < 90) {
                const n0 = 1.0 + 0.000293 * (station3D.pres / 1013.25) * (273.15 / (273.15 + station3D.temp));
                let angleZ = (90 - elDeg) * Math.PI / 180.0;
                for (let couche = 10; couche >= 1; couche--) {
                    let n_couche = 1.0 + (n0 - 1.0) * (couche / 10.0);
                    let n_dessus = 1.0 + (n0 - 1.0) * ((couche - 1) / 10.0);
                    angleZ = Math.asin((n_dessus / n_couche) * Math.sin(angleZ));
                }
                elRefractee = 90.0 - (angleZ * 180.0 / Math.PI);
                inflexionStr = (elRefractee - elDeg).toFixed(4) + "°";
            }

            resultats[astre] = { azVrai: azDeg, elRefractee, distanceUA: distM / UA_TO_M, inflexion: inflexionStr };
        });
        postMessage({ status: 'OK', data: resultats });
    } catch (err) {
        postMessage({ status: 'ERREUR', message: err.toString() });
    }
};
