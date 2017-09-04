/* Produced by CVXGEN, 2017-09-04 14:14:14 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
  double time;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  settings.max_iters = 10;
  settings.eps = 1e-3;
  settings.resid_tol = 1e-3;
  tic();
  num_iters = solve();
  time = tocq();
  printf("Timed %d solves over %.6f seconds.\n", NUMTESTS, time);
  printf("f = %f, %f, %f, %f\n", vars.f[0], vars.f[1], vars.f[2], vars.f[3]);
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.W_row3[0] = -0.54176666;
  params.W_row3[1] = -0.73584623;
  params.W_row3[2] = -0.5300961;
  params.W_row3[3] = -0.33601652;
  params.wdes[0] = -2.4503629212121192;
  params.wdes[1] = 9.86480390e-04;
  params.wdes[2] = -1.16191441e-03;
  params.wdes[3] = 6.15324671e-06;
  params.W_row2[0] = -0.19991486;
  params.W_row2[1] = 0.00583528;
  params.W_row2[2] = 0.19991486;
  params.W_row2[3] = -0.00583528;
  params.W_row4[0] = 0.02;
  params.W_row4[1] = -0.02;
  params.W_row4[2] = 0.02;
  params.W_row4[3] = -0.02;
  params.FMIN[0] = -2.0;
  params.FMAX[0] = 0.0;
}

