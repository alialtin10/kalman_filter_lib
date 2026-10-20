#include "test_helpers.hpp"

#include <gtest/gtest.h>

using kfl::models::DifferentialDriveModel;

// Doğrusal sistemde EKF, standart KF ile birebir aynı sonucu vermeli
TEST(ExtendedKalmanFilter, MatchesKalmanFilterOnLinearSystem)
{
    const kfl_test::ConstantVelocity cv(0.1, 0.5, 0.25);
    kfl::KalmanFilter<2,1> kf;
    kf.setSystemMatrices(cv.F, cv.Q, cv.H, cv.R);

    kfl::ExtendedKalmanFilter<2> ekf(std::make_shared<kfl::LinearSystemModel<2>>(cv.F, cv.Q));
    const kfl::LinearMeasurementModel<2,1> meas(cv.H, cv.R);

    const Eigen::Vector2d x0(1.0, -0.5);
    const Eigen::Matrix2d P0 = Eigen::Vector2d(4.0, 1.0).asDiagonal();
    kf.initialize(x0, P0);
    ekf.initialize(x0, P0);

    std::mt19937 rng(3);
    std::normal_distribution<double> n01(0.0, 1.0);
    for (int k = 0; k < 100; ++k)
    {
        const Eigen::Matrix<double,1,1> z(0.1 * k + n01(rng));
        kf.predict();
        ekf.predict();
        kf.update(z);
        ekf.update(meas, z);
    }
    EXPECT_TRUE(ekf.getState().isApprox(kf.getState(), 1e-12));
    EXPECT_TRUE(ekf.getCovariance().isApprox(kf.getCovariance(), 1e-12));
}

TEST(ExtendedKalmanFilter, RejectsNullModel)
{
    using Ekf = kfl::ExtendedKalmanFilter<3,2>;
    EXPECT_THROW(Ekf(std::shared_ptr<const Ekf::Model>()), std::invalid_argument);
}

TEST(ExtendedKalmanFilter, DiffDriveWithGpsBeatsDeadReckoning)
{
    kfl_test::DiffDriveScenario sc;
    sc.sigma_v = 0.1;
    sc.sigma_omega = 0.05;
    sc.odom_v_scale = 1.03;
    sc.odom_omega_bias = 0.02;
    // Q, modellenmemiş sistematik sapmayı da kapsayacak şekilde büyütüldü (tuning)
    auto model = std::make_shared<DifferentialDriveModel>(sc.dt, sc.sigma_v, 0.4);
    kfl::ExtendedKalmanFilter<3,2> ekf(model);

    const auto res = kfl_test::runDiffDrive(ekf, *model, sc);
    EXPECT_LT(res.pos_rmse, 0.8);
    EXPECT_LT(res.pos_rmse, 0.5 * res.dead_reckoning_rmse);
}

TEST(ExtendedKalmanFilter, DiffDriveWithLandmarksIsAccurate)
{
    kfl_test::DiffDriveScenario sc;
    sc.use_gps = false;
    sc.use_landmarks = true;
    auto model = std::make_shared<DifferentialDriveModel>(sc.dt, sc.sigma_v, sc.sigma_omega);
    kfl::ExtendedKalmanFilter<3,2> ekf(model);

    const auto res = kfl_test::runDiffDrive(ekf, *model, sc);
    EXPECT_LT(res.pos_rmse, 0.1);
    EXPECT_LT(res.heading_rmse, 0.02);
}

TEST(ExtendedKalmanFilter, GatingRejectsGpsOutliers)
{
    kfl_test::DiffDriveScenario sc;
    sc.gps_outlier_prob = 0.15;
    auto model = std::make_shared<DifferentialDriveModel>(sc.dt, sc.sigma_v, sc.sigma_omega);

    kfl::ExtendedKalmanFilter<3,2> gated(model);
    sc.nis_gate = 9.21;  // ki-kare, 2 DoF, %99
    const auto with_gate = kfl_test::runDiffDrive(gated, *model, sc);

    kfl::ExtendedKalmanFilter<3,2> ungated(model);
    sc.nis_gate = kfl::FilterBase<3>::kNoGate;
    const auto without_gate = kfl_test::runDiffDrive(ungated, *model, sc);

    EXPECT_GT(with_gate.rejected, 0);
    EXPECT_LT(with_gate.pos_rmse, 0.7 * without_gate.pos_rmse);
}

// Robot sürekli döndüğü için yön açısı defalarca +-pi sınırını geçer
TEST(ExtendedKalmanFilter, HeadingWrapsAroundPi)
{
    kfl_test::DiffDriveScenario sc;
    sc.initial_heading = 3.1;
    sc.use_gps = false;
    sc.use_landmarks = true;
    auto model = std::make_shared<DifferentialDriveModel>(sc.dt, sc.sigma_v, sc.sigma_omega);
    kfl::ExtendedKalmanFilter<3,2> ekf(model);

    const auto res = kfl_test::runDiffDrive(ekf, *model, sc);
    EXPECT_TRUE(res.heading_always_normalized);
    EXPECT_LT(res.heading_rmse, 0.02);
}
