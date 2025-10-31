/*
 (c) 2011-2015, Vladimir Agafonkin
 SunCalc is a JavaScript library for calculating sun/moon positions and phases.
 https://github.com/mourner/suncalc
*/

(function () { 'use strict';

// date/time constants and conversions

var dayMs = 1000 * 60 * 60 * 24,
    J1970 = 2440588,
    J2000 = 2451545;

function toJulian(date) { return date.valueOf() / dayMs - 0.5 + J1970; }
function fromJulian(j) { return new Date((j + 0.5 - J1970) * dayMs); }
function toDays(date) { return toJulian(date) - J2000; }


// general calculations for position

var rad = Math.PI / 180,
    e = rad * 23.4397; // obliquity of the Earth's axis

function rightAscension(l, b) { return Math.atan2(Math.sin(l) * Math.cos(e) - Math.tan(b) * Math.sin(e), Math.cos(l)); }
function declination(l, b) { return Math.asin(Math.sin(b) * Math.cos(e) + Math.cos(b) * Math.sin(e) * Math.sin(l)); }

function azimuth(H, phi, dec) { return Math.atan2(Math.sin(H), Math.cos(H) * Math.sin(phi) - Math.tan(dec) * Math.cos(phi)); }
function altitude(H, phi, dec) { return Math.asin(Math.sin(phi) * Math.sin(dec) + Math.cos(phi) * Math.cos(dec) * Math.cos(H)); }

function siderealTime(d, lw) { return rad * (280.16 + 360.98564737 * d) - lw; }

function astroRefraction(h) {
    if (h < 0) // the formula is not generally accurate near horizon
        h = 0;

    // formula 16.4 of "Astronomical Algorithms" 2nd edition by Jean Meeus (page 111)
    return 0.0002967 / Math.tan(h + 0.00312536 / (h + 0.3477777));
}

// sun calculations

function solarMeanAnomaly(d) { return rad * (356.0470 + 0.9856002585 * d); }

function eclipticLongitude(M) {
    var C = rad * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), // equation of center
        P = rad * 102.9377;                                                                // perihelion of the Earth

    return M + C + P + Math.PI;
}

function sunCoords(d) {

    var M = solarMeanAnomaly(d),
        L = eclipticLongitude(M);

    return {
        dec: declination(L, 0),
        ra: rightAscension(L, 0)
    };
}

var SunCalc = {};

// calculates sun position for a given date and latitude/longitude

SunCalc.getPosition = function (date, lat, lon) {

    var lw  = rad * -lon,
        phi = rad * lat,
        d   = toDays(date),

        c  = sunCoords(d),
        H  = siderealTime(d, lw) - c.ra,
        alt = altitude(H, phi, c.dec); // altitude with refraction

    return {
        azimuth: azimuth(H, phi, c.dec),
        altitude: alt + astroRefraction(alt)
    };
};

// sun times configuration (angle, morning name, evening name)

var times = SunCalc.times = [
    [-0.833, 'sunrise',       'sunset'      ],
    [ -0.3,  'sunriseEnd',    'sunsetStart' ],
    [ -6,    'dawn',          'dusk'        ],
    [-12,    'nauticalDawn',  'nauticalDusk'],
    [-18,    'nightEnd',      'night'       ],
    [ 6,     'goldenHourEnd', 'goldenHour'  ]
];

// adds a custom time to the times config

SunCalc.addTime = function (angle, riseName, setName) {
    times.push([angle, riseName, setName]);
};


// calculations for sun times

var J0 = 0.0009;

function julianCycle(d, lw) { return Math.round(d - J0 - lw / (2 * Math.PI)); }

function approxTransit(Ht, lw, n) { return J0 + (Ht + lw) / (2 * Math.PI) + n; }

function solarTransit(ds, M, L) { return J2000 + ds + 0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * L); }

function hourAngle(h, phi, dec) {
    return Math.acos((Math.sin(h) - Math.sin(phi) * Math.sin(dec)) / (Math.cos(phi) * Math.cos(dec)));
}

// returns set time for the given sun altitude
function getSetJ(h, lw, phi, dec, n, M, L) {

    var w = hourAngle(h, phi, dec),
        a = approxTransit(w, lw, n);
    return solarTransit(a, M, L);
}


// calculates sun times for a given date and latitude/longitude

SunCalc.getTimes = function (date, lat, lon) {

    var lw  = rad * -lon,
        phi = rad * lat,
        d   = toDays(date),

        n  = julianCycle(d, lw),
        ds = approxTransit(0, lw, n),

        M = solarMeanAnomaly(ds),
        L = eclipticLongitude(M),
        c = sunCoords(ds),

        Jnoon = solarTransit(ds, M, L),

        result = {
            solarNoon: fromJulian(Jnoon),
            nadir: fromJulian(Jnoon + 0.5)
        };


    for (var i = 0; i < times.length; i += 1) {
        
        var h0 = times[i][0] * rad,
            Jset = getSetJ(h0, lw, phi, c.dec, n, M, L),
            Jrise = Jnoon - (Jset - Jnoon);

        result[times[i][1]] = fromJulian(Jrise);
        result[times[i][2]] = fromJulian(Jset);
    }

    return result;
};


// moon calculations, based on http://aa.quae.nl/en/reken/maanpositie.html formulas

function moonCoords(d) { // geocentric ecliptic coordinates of the moon

    var L = rad * (218.316 + 13.176396 * d), // ecliptic longitude
        M = rad * (134.963 + 13.064993 * d), // mean anomaly
        F = rad * (93.272 + 13.229350 * d), // mean distance

        l  = L + rad * 6.289 * Math.sin(M), // longitude
        b  = rad * 5.128 * Math.sin(F),     // latitude
        dt = 385001 - 20905 * Math.cos(M);  // distance to the Earth in km

    return {
        ra: rightAscension(l, b),
        dec: declination(l, b),
        dist: dt
    };
}

SunCalc.getMoonPosition = function (date, lat, lon) {

    var lw  = rad * -lon,
        phi = rad * lat,
        d   = toDays(date),

        c = moonCoords(d),
        H = siderealTime(d, lw) - c.ra,
        alt = altitude(H, phi, c.dec),
        az = azimuth(H, phi, c.dec);

    return {
        azimuth: az,
        altitude: alt,
        distance: c.dist
    };
};

// calculations for illumination parameters of the moon,
// based on http://idlastro.gsfc.nasa.gov/ftp/pro/astro/mphase.pro formulas and
// Chapter 48 of "Astronomical Algorithms" 2nd edition by Jean Meeus (page 343)

SunCalc.getMoonIllumination = function (date) {

    var d = toDays(date),
        s = sunCoords(d),
        m = moonCoords(d),

        sdist = 149598000, // distance from Earth to Sun in km

        phi = Math.acos(Math.sin(s.dec) * Math.sin(m.dec) + Math.cos(s.dec) * Math.cos(m.dec) * Math.cos(s.ra - m.ra)),
        inc = Math.atan2(sdist * Math.sin(phi), m.dist - sdist * Math.cos(phi)),
        angle = Math.atan2(Math.cos(s.dec) * Math.sin(s.ra - m.ra), Math.sin(s.dec) * Math.cos(m.dec) -
                           Math.cos(s.dec) * Math.sin(m.dec) * Math.cos(s.ra - m.ra));

    return {
        fraction: (1 + Math.cos(inc)) / 2,
        phase: 0.5 + 0.5 * inc * (angle < 0 ? -1 : 1) / Math.PI,
        angle: angle
    };
};


function hoursLater(date, h) {
    return new Date(date.valueOf() + h * dayMs / 24);
}

// calculations for moon rise/set times are based on http://www.stargazing.net/kepler/moonrise.html article

SunCalc.getMoonTimes = function (date, lat, lon, inUtc) {
    var hc = 0.133 * rad,
        h0 = SunCalc.getMoonPosition(date, lat, lon).altitude - hc,
        
        t = inUtc ? date : new Date(date.getTime() + (date.getTimezoneOffset() * 60000)),
        
        d = toDays(t),
        lw  = rad * -lon,
        phi = rad * lat,
        
        n = julianCycle(d, lw),
        ds = approxTransit(0, lw, n),
        
        c = moonCoords(ds),
        
        H = siderealTime(ds, lw) - c.ra,
        
        Jrise = NaN,
        Jset = NaN;

    if (h0 < 0) {
        
        var w = hourAngle(hc, phi, c.dec),
            a = approxTransit(w, lw, n),
            
            J1 = solarTransit(a, 0, 0),
            J2 = J1 + (J1 < ds ? 1 : 0),
            
            J3 = Jnoon - (J2 - Jnoon);

        if (J3 > J2) {
            Jrise = Jnoon - (J1 - Jnoon);
            Jset = J1;
        } else {
            Jrise = J3;
            Jset = J2;
        }

    }

    return {
        rise: isNaN(Jrise) ? null : fromJulian(Jrise),
        set: isNaN(Jset) ? null : fromJulian(Jset)
    };
};

// export as Node module / AMD module / browser variable
if (typeof exports === 'object' && typeof module !== 'undefined') module.exports = SunCalc;
else if (typeof define === 'function' && define.amd) define(SunCalc);
else window.SunCalc = SunCalc;

})();
