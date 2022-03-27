#include "../catch.hpp"
#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>

struct ResidualStruct {
  ResidualStruct(double x, double y) : x_(x), y_(y) {}
  template <typename T> bool operator()(const T *const abc, T *res) const {
    res[0] =
        T(y_) - ceres::exp(abc[0] * T(x_) * T(x_) + abc[1] * T(x_) + abc[2]);
    return true;
  }

private:
  double x_;
  double y_;
};

struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

  // 残差的计算
  template <typename T>
  bool operator()(const T *const abc, // 模型参数，有3维
                  T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) +
                                     abc[2]); // y-exp(ax^2+bx+c)
    return true;
  }

  const double _x, _y; // x,y数据
};

TEST_CASE("CERES", "TesT") {
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
        ceres::exp(a * x * x + b * x + c) + rng.gaussian(w_sigma * w_sigma);
    x_data.push_back(x);
    y_data.push_back(y);
  }

  double abc[3] = {ae, be, ce};
  ceres::Problem problem;
  for (size_t i = 0; i < N_DATA; i++) {
    /* code */
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ResidualStruct, 1, 3>(
            new ResidualStruct(x_data[i], y_data[i])),
        nullptr, abc);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;
  std::cout << "estimated a,b,c = ";
  for (auto a : abc) {
    std::cout << a << " ";
    std::cout << std::endl;
  }
}