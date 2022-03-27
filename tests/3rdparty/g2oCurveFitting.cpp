#include "../catch.hpp"
#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std;

class CFVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void setToOriginImpl() override { _estimate << 0, 0, 0; }
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);
  }
  virtual bool read(istream &in) {}
  virtual bool write(ostream &out) const {}
};

class CFEdge : public g2o::BaseUnaryEdge<1, double, CFVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CFEdge(double x) : BaseUnaryEdge(), _x(x) {}
  virtual void computeError() override {
    const CFVertex *v = static_cast<const CFVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0, 0) =
        _measurement - exp(abc(0, 0) * _x * _x + abc(1) * _x + abc(2));
  }
  virtual void linearizeOplus() override {
    const CFVertex *v = static_cast<const CFVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double y = exp(abc(0) * _x * _x + abc(1) * _x + abc(2));
    _jacobianOplusXi[0] = -_x * _x * y;
    _jacobianOplusXi[1] = -_x * y;
    _jacobianOplusXi[2] = -y;
  }
  virtual bool read(istream &in) {}
  virtual bool write(ostream &out) const {}

private:
  double _x;
};

TEST_CASE("G2o", "CurveFitting") {
  double a = 1.0;
  double b = 2.0;
  double c = 1.0;

  double ae = 2.0;
  double be = -1.0;
  double ce = 5.0;

  constexpr int N_ITERATIONS = 1000;
  constexpr int N_DATA = 1000;
  double w_sigma = 1.0;
  cv::RNG rng;

  std::vector<double> x_data, y_data;

  for (size_t i = 0; i < N_DATA; i++) {
    double x = i / 1000.0;
    double y =
        std::exp(a * x * x + b * x + c) + rng.gaussian(w_sigma * w_sigma);
    x_data.push_back(x);
    y_data.push_back(y);
  }

  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
  using LinearSolverType =
      g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);
  CFVertex *v = new CFVertex();
  v->setEstimate(Eigen::Vector3d(ae, be, ce));
  v->setId(0);
  optimizer.addVertex(v);

  for (int i = 0; i < N_DATA; i++) {
    CFEdge *edge = new CFEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(y_data[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 /
                         (w_sigma * w_sigma));
    optimizer.addEdge(edge);
  }

  cout << "start optimization" << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used =
      chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  Eigen::Vector3d abc_estimate = v->estimate();
  cout << "estimated model: " << abc_estimate.transpose() << endl;
}