#include <foundation/newton_raphson.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace {

struct EvalFunction {
  double operator()(double x) const { return x * x * x - 2 * x * x + 1; }
  double derivative(double x) const { return 3 * x * x - 4 * x; }
};
};  // namespace

TEST(NewtonRaphson, ConvergesWithFiniteDiff) {
  const int iterations = 20;
  const double tolerance = 1e-10;
  const double initial_value = 0.1;

  EvalFunction eval_function;
  const auto root =
      foundation::NewtonRaphson<EvalFunction, 1, 1,
                                foundation::FiniteDiff<EvalFunction, 1, 1>>(
          eval_function, initial_value, iterations, tolerance);

  EXPECT_NEAR(1.61803, root, 1e-5);
}

TEST(NewtonRaphson, ConvergesWithManualDiff) {
  const int iterations = 20;
  const double tolerance = 1e-10;
  const double initial_value = 0.1;

  EvalFunction eval_function;
  const auto root =
      foundation::NewtonRaphson<EvalFunction, 1, 1,
                                foundation::ManualDiff<EvalFunction, 1, 1>>(
          eval_function, initial_value, iterations, tolerance);

  EXPECT_NEAR(1.61803, root, 1e-5);
}
