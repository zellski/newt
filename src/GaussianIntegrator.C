# include "GaussianIntegrator.h"
# include "Stage.h"

# include <assert.h>
# include <stdio.h>

GaussianIntegrator::GaussianIntegrator(int nn) : n(nn) {
    assert(n == 5 || n == 8 || n == 10);

    weights = new double[n];
    points  = new double[n];

    if (n == 5) {
        double w0 = 0.56888889;

        double d1 = 0.53846931;     double w1 = 0.47862867;
        double d2 = 0.90617985;     double w2 = 0.23692689;

        points[0] = 0.5 - d2/2.0;   weights[0] = w2;
        points[1] = 0.5 - d1/2.0;   weights[1] = w1;
        points[2] = 0.5;            weights[2] = w0;
        points[3] = 0.5 + d1/2.0;   weights[3] = w1;
        points[4] = 0.5 + d2/2.0;   weights[4] = w2;
    } else if (n == 8) {
        double d1 = 0.18343464;  double w1 = 0.36268378;
        double d2 = 0.52553241;  double w2 = 0.31370665;
        double d3 = 0.79666648;  double w3 = 0.22238103;
        double d4 = 0.96028986;  double w4 = 0.10122854;

        points[0] = 0.5 - d4/2.0;   weights[0] = w4;
        points[1] = 0.5 - d3/2.0;   weights[1] = w3;
        points[2] = 0.5 - d2/2.0;   weights[2] = w2;
        points[3] = 0.5 - d1/2.0;   weights[3] = w1;
        points[4] = 0.5 + d1/2.0;   weights[4] = w1;
        points[5] = 0.5 + d2/2.0;   weights[5] = w2;
        points[6] = 0.5 + d3/2.0;   weights[6] = w3;
        points[7] = 0.5 + d4/2.0;   weights[7] = w4;
    } else if (n == 10) {
        double d1 = 0.14887434; double w1 = 0.29552422;
        double d2 = 0.43339539; double w2 = 0.26926672;
        double d3 = 0.67940957; double w3 = 0.21908636;
        double d4 = 0.86506337; double w4 = 0.14945135;
        double d5 = 0.97390653; double w5 = 0.06667134;

        points[0] = 0.5 - d5/2.0;   weights[0] = w5;
        points[1] = 0.5 - d4/2.0;   weights[1] = w4;
        points[2] = 0.5 - d3/2.0;   weights[2] = w3;
        points[3] = 0.5 - d2/2.0;   weights[3] = w2;
        points[4] = 0.5 - d1/2.0;   weights[4] = w1;
        points[5] = 0.5 - d1/2.0;   weights[5] = w1;
        points[6] = 0.5 + d2/2.0;   weights[6] = w2;
        points[7] = 0.5 + d3/2.0;   weights[7] = w3;
        points[8] = 0.5 + d4/2.0;   weights[8] = w4;
        points[9] = 0.5 + d5/2.0;   weights[9] = w5;
    }
}


void GaussianIntegrator::integrate(Stage *S,
                                  const int slice,
                                  const adoublev &x,
                                  adoublev &c,
                                  adouble &f0) {
    for (int sample = 0; sample < n; sample ++) {
        const double weight = weights[sample]/2;
        const double t = points[sample];
        S->FEMPoint(slice, t, weight, x, c, f0);
    }
}
