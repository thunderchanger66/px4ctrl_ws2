#include <stdexcept>

#include <gtest/gtest.h>

#include "traj_utils/polynomial_traj.h"

TEST(PolynomialTrajectory, SingleSegmentHonorsBoundaryConditions)
{
  Eigen::Matrix<double, 3, 2> points;
  points << 1.0, 3.0, -2.0, 4.0, 0.5, 1.5;
  const Eigen::Vector3d v0(0.1, 0.2, 0.3), v1(-0.1, 0.3, 0.2);
  const Eigen::Vector3d a0(0.0, 0.2, 0.1), a1(-0.1, 0.0, 0.2);
  const Eigen::VectorXd duration = Eigen::VectorXd::Constant(1, 2.0);
  auto trajectory = PolynomialTraj::minSnapTraj(points, v0, v1, a0, a1, duration);
  trajectory.init();
  EXPECT_TRUE(trajectory.evaluate(0.0).isApprox(points.col(0), 1e-9));
  EXPECT_TRUE(trajectory.evaluate(2.0).isApprox(points.col(1), 1e-9));
  EXPECT_TRUE(trajectory.evaluateVel(0.0).isApprox(v0, 1e-9));
  EXPECT_TRUE(trajectory.evaluateVel(2.0).isApprox(v1, 1e-9));
  EXPECT_TRUE(trajectory.evaluateAcc(0.0).isApprox(a0, 1e-9));
  EXPECT_TRUE(trajectory.evaluateAcc(2.0).isApprox(a1, 1e-9));
}

TEST(PolynomialTrajectory, RejectsZeroDuration)
{
  const Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 2);
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  EXPECT_THROW(PolynomialTraj::minSnapTraj(
      points, zero, zero, zero, zero, Eigen::VectorXd::Zero(1)), std::invalid_argument);
}
