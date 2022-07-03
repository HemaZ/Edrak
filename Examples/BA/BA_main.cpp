#include "SnavelyReprojectionError.h"
#include "common.hpp"
#include <ceres/ceres.h>
void SolveBA(BALProblem &bal_problem);

int main(int argc, char const *argv[]) {
  std::string data_dir = EDRAK_TEST_DATA_DIR;

  BALProblem baProblem(data_dir + "problem-16-22106-pre.txt");
  baProblem.Normalize();
  baProblem.Perturb(0.1, 0.5, 0.5);
  baProblem.WriteToPLYFile("initial.ply");
  SolveBA(baProblem);
  baProblem.WriteToPLYFile("final.ply");

  return 0;
}

void SolveBA(BALProblem &bal_problem) {
  const int point_block_size = bal_problem.point_block_size();
  const int camera_block_size = bal_problem.camera_block_size();
  double *points = bal_problem.mutable_points();
  double *cameras = bal_problem.mutable_cameras();

  // Observations is 2 * num_observations long array observations
  // [u_1, u_2, ... u_n], where each u_i is two dimensional, the x
  // and y position of the observation.
  const double *observations = bal_problem.observations();
  ceres::Problem problem;

  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    ceres::CostFunction *cost_function;

    // Each Residual block takes a point and a camera as input
    // and outputs a 2 dimensional Residual
    cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0],
                                                     observations[2 * i + 1]);

    // If enabled use Huber's loss function.
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    // Each observation corresponds to a pair of a camera and a point
    // which are identified by camera_index()[i] and point_index()[i]
    // respectively.
    double *camera =
        cameras + camera_block_size * bal_problem.camera_index()[i];
    double *point = points + point_block_size * bal_problem.point_index()[i];

    problem.AddResidualBlock(cost_function, loss_function, camera, point);
  }

  // show some information here ...
  std::cout << "bal problem file loaded..." << std::endl;
  std::cout << "bal problem have " << bal_problem.num_cameras()
            << " cameras and " << bal_problem.num_points() << " points. "
            << std::endl;
  std::cout << "Forming " << bal_problem.num_observations() << " observations. "
            << std::endl;

  std::cout << "Solving ceres BA ... " << std::endl;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}
