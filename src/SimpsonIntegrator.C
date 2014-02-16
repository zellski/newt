# include "SimpsonIntegrator.h"
# include "Stage.h"

# include <assert.h>
# include <stdio.h>

SimpsonIntegrator::SimpsonIntegrator(int nn) : n(nn) {
    assert((n % 2) == 1);
    assert(n < 100);

    weights = new double[nn];
    points  = new double[nn];

    for (int i = 0; i < n; i ++) {
        points[i] = (double) i / (double) (n-1);
        if ((i % 2) == 0) {
            weights[i] = 2;
        } else {
            weights[i] = 4;
        }
        fprintf(stderr, "%d: %f\n", i, points[i]);
    }
    weights[0] = weights[n-1] = 1;
}

void SimpsonIntegrator::integrate(Stage *S,
                                  const int slice,
                                  const adoublev &x,
                                  adoublev &c,
                                  adouble &f0) {
    for (int sample = 0; sample < n; sample ++) {
        const double weight = weights[sample]/n/3;
        const double t = points[sample];
        S->FEMPoint(slice, t, weight, x, c, f0);
    }
}
