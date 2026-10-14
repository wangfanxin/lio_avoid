// Reference evaluator: the PLANNER'S OWN `ego_planner::UniformBspline`, compiled straight from
// bspline_opt/src/uniform_bspline.cpp and driven from a text file.
//
// This exists so that `planner/ego_adapter.py`'s Python transliteration of `evaluateDeBoorT` can be
// checked against the real C++ on arbitrary data instead of against a re-reading of the same source.
// tools/ego-bspline-ab.py builds the input, runs both and prints the largest disagreement.
//
// input (whitespace separated):
//     order n_ctrl n_eval
//     u[0] .. u[n_ctrl + order]                  (n_ctrl + order + 1 knots)
//     n_ctrl lines of "x y z"
//     n_eval values of t
// output: n_eval lines of "x y z"
#include "bspline_opt/uniform_bspline.h"

#include <Eigen/Eigen>
#include <cstdio>
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: bspline-ref <input.txt>\n";
    return 2;
  }
  FILE *f = fopen(argv[1], "r");
  if (!f)
  {
    std::cerr << "cannot open " << argv[1] << "\n";
    return 2;
  }
  int order = 0, n_ctrl = 0, n_eval = 0;
  if (fscanf(f, "%d %d %d", &order, &n_ctrl, &n_eval) != 3)
  {
    std::cerr << "bad header\n";
    return 2;
  }

  int nknot = n_ctrl + order + 1;
  Eigen::VectorXd knots(nknot);
  for (int i = 0; i < nknot; ++i)
    if (fscanf(f, "%lf", &knots(i)) != 1)
    {
      std::cerr << "bad knot " << i << "\n";
      return 2;
    }

  Eigen::MatrixXd pts(3, n_ctrl);
  for (int i = 0; i < n_ctrl; ++i)
    if (fscanf(f, "%lf %lf %lf", &pts(0, i), &pts(1, i), &pts(2, i)) != 3)
    {
      std::cerr << "bad control point " << i << "\n";
      return 2;
    }
  fclose(f);

  // exactly what plan_manage/src/traj_server.cpp does with a traj_utils/Bspline message
  ego_planner::UniformBspline traj(pts, order, 0.1);
  traj.setKnot(knots);

  FILE *g = fopen(argv[2], "r");
  if (!g)
  {
    std::cerr << "cannot open " << argv[2] << "\n";
    return 2;
  }
  for (int i = 0; i < n_eval; ++i)
  {
    double t = 0.0;
    if (fscanf(g, "%lf", &t) != 1)
    {
      std::cerr << "bad t " << i << "\n";
      return 2;
    }
    Eigen::VectorXd p = traj.evaluateDeBoorT(t);
    printf("%.17g %.17g %.17g\n", p(0), p(1), p(2));
  }
  fclose(g);
  return 0;
}
