#include "test_helpers.hpp"

#include <gtest/gtest.h>

using kfl::utils::kPi;
using kfl::utils::normalizeAngle;

TEST(MathUtils, NormalizeAngleWrapsIntoRange)
{
    EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
    EXPECT_NEAR(normalizeAngle(3.0 * kPi / 2.0), -kPi / 2.0, 1e-12);
    EXPECT_NEAR(normalizeAngle(-3.0 * kPi / 2.0), kPi / 2.0, 1e-12);
    EXPECT_NEAR(normalizeAngle(10.0 * kPi + 0.1), 0.1, 1e-9);
    EXPECT_NEAR(normalizeAngle(kPi), -kPi, 1e-12);  // [-pi, pi) aralığı
}

TEST(MathUtils, NumericalJacobianOfLinearFunctionIsExact)
{
    Eigen::Matrix<double,2,3> A;
    A << 1, 2, 3,
         4, 5, 6;
    const auto J = kfl::utils::numericalJacobian<2,3>(
        [&](const Eigen::Vector3d& x) { return Eigen::Vector2d(A * x); }, Eigen::Vector3d(0.3, -1.0, 2.0));
    EXPECT_TRUE(J.isApprox(A, 1e-8));
}

TEST(DifferentialDriveModel, AnalyticJacobianMatchesNumerical)
{
    const kfl::models::DifferentialDriveModel model(0.05, 0.1, 0.05);
    const Eigen::Vector3d x(1.0, -2.0, 2.9);
    const Eigen::Vector2d u(1.5, 0.8);

    const auto analytic = model.F(x, u);
    // Taban sınıfın sayısal Jacobian'ını açıkça çağır
    const auto numeric = model.kfl::SystemModel<3,2>::F(x, u);
    EXPECT_TRUE(analytic.isApprox(numeric, 1e-6)) << analytic << "\n\n" << numeric;
}

TEST(RangeBearingMeasurement, AnalyticJacobianMatchesNumerical)
{
    const kfl::models::RangeBearingMeasurement lm(4.0, 3.0, 0.1, 0.01);
    // Yön ölçümünün +-pi sınırına yakın olduğu bir durum: residual() sarmalı fark kullanmalı
    const Eigen::Vector3d x(5.0, 3.0001, 0.0);

    const auto analytic = lm.H(x);
    const auto numeric = lm.kfl::MeasurementModel<3,2>::H(x);
    EXPECT_TRUE(analytic.isApprox(numeric, 1e-5)) << analytic << "\n\n" << numeric;
}

TEST(DifferentialDriveModel, StraightLineAndInPlaceRotation)
{
    const kfl::models::DifferentialDriveModel model(1.0, 0.0, 0.0);
    const Eigen::Vector3d straight = model.f(Eigen::Vector3d(0, 0, kPi / 2), Eigen::Vector2d(2.0, 0.0));
    EXPECT_TRUE(straight.isApprox(Eigen::Vector3d(0.0, 2.0, kPi / 2), 1e-12));

    const Eigen::Vector3d rotated = model.f(Eigen::Vector3d(1, 1, 3.0), Eigen::Vector2d(0.0, 1.0));
    EXPECT_NEAR(rotated(0), 1.0, 1e-12);
    EXPECT_NEAR(rotated(1), 1.0, 1e-12);
    EXPECT_NEAR(rotated(2), normalizeAngle(4.0), 1e-12);
}
