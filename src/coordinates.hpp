#include <cfloat>
#include <cmath>

#define UTS_TO_JDN(ts) (float)((float)(ts) / 86400 + 2440587.5)
#define J2000 2451545.0f
#define J1950 2433282.4235f

#ifndef radians
#define DEG_TO_RAD 0.017453292519943295769236907684886  // PI / 180
#define RAD_TO_DEG 57.295779513082320876798154814105    // 180 / PI
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#endif

#define d_isclose(a, b) (fabsf((a) - (b)) < FLT_EPSILON)

typedef struct __attribute__((packed)) {
    float ra, dec;     // seconds
    float epoch;       // JDN
} eq_coord_t;

void _prmat(eq_coord_t &from, eq_coord_t &to) {
    double t, alf, del;

    if (d_isclose(from.epoch, J1950))
        t = to.epoch - J1950;
    else
        t = from.epoch - to.epoch;
    t /= 36525.0;

    double d0 = 0.23449210459913079630223597777156799e-7;
    double d1 = 0.11174959386818911539397789815793604e-1;
    double d2 = 0.14651700808223472170092316405645731e-5;
    double d3 = 0.87272054493620787482503185255845250e-7;
    double t0 = 0.20390124985524953491693911357919683e-7;
    double t1 = 0.97171211299597746038990513165103640e-2;
    double t2 = -0.20692744591763537661702895036771455e-5;
    double t3 = -0.20266510432407493982047961909412954e-6;
    double z0 = 0.23449210476839938739538526785799463e-7;
    double z1 = 0.11174959402952244765215142272698626e-1;
    double z2 = 0.53094219810269135628994192263211485e-5;
    double z3 = 0.88726588735181133940544905010109307e-7;
    double dzeta = d0 + t * (d1 + t * (d2 + t * d3));
    double theta = t0 + t * (t1 + t * (t2 + t * t3));
    double zet = z0 + t * (z1 + t * (z2 + t * z3));
    double c0;
    double c1 = cos(dzeta);
    double c2 = cos(zet);
    double c3 = cos(theta);
    double s1 = sin(dzeta);
    double s2 = sin(zet);
    double s3 = sin(theta);
    double p00 = c1 * c2 * c3 - s1 * s2;
    double p01 = -1.0 * s1 * c2 * c3 - c1 * s2;
    double p02 = -1.0 * c2 * s3;
    double p10 = c1 * s2 * c3 + s1 * c2;
    double p11 = c1 * c2 - s1 * s2 * c3;
    double p12 = -1.0 * s2 * s3;
    double p20 = c1 * s3;
    double p21 = -1.0 * s1 * s3;
    double p22 = c3;

    alf = from.ra;
    del = from.dec;

    double x0 = cos(alf) * cos(del);
    double x1 = sin(alf) * cos(del);
    double x2 = sin(del);

    if (d_isclose(from.epoch, J1950)) {
        c0 = p00 * x0 + p01 * x1 + p02 * x2;
        c1 = p10 * x0 + p11 * x1 + p12 * x2;
        c2 = p20 * x0 + p21 * x1 + p22 * x2;
    } else {
        c0 = p00 * x0 + p10 * x1 + p20 * x2;
        c1 = p01 * x0 + p11 * x1 + p21 * x2;
        c2 = p02 * x0 + p12 * x1 + p22 * x2;
    }

    double ra = atan2(c1, c0);
    if (ra < 0.0)
        ra = ra + M_PI * 2;
    to.ra = (float)ra;
    to.dec = (float)atan2(c2, sqrt(c0 * c0 + c1 * c1));
}

void precession_correction(eq_coord_t from, eq_coord_t& to) {
    if (from.epoch == 0)
        from.epoch = J2000;

    // convert to radians
    from.ra = radians((from.ra * 15.0) / 3600.0);
    from.dec = radians((from.dec) / 3600.0);

    eq_coord_t e_1950{0, 0, J1950};
    _prmat(from, e_1950);
    _prmat(e_1950, to);

    to.ra = degrees(to.ra) * 3600.0 / 15.0;
    to.dec = degrees(to.dec) * 3600.0;
}
