#include "test_helpers.hpp"

#include <gtest/gtest.h>

using kfl::models::DifferentialDriveModel;

// Doğrusal-Gauss sistemde unscented dönüşüm kesindir: UKF, KF ile aynı sonucu vermeli
TEST(UnscentedKalmanFilter, MatchesKalmanFilterOnLinearSystem)
{
    const kfl_test::ConstantVelocity cv(0.1, 0.5, 0.25);
    kfl::KalmanFilter<2,1> kf;
    kf.setSystemMatrices(cv.F, cv.Q, cv.H, cv.R);

    kfl::UnscentedKalmanFilter<2> ukf(std::make_shared<kfl::LinearSystemModel<2>>(cv.F, cv.Q));
    const kfl::LinearMeasurementModel<2,1> meas(cv.H, cv.R);

    const Eigen::Vector2d x0(1.0, -0.5);
    const Eigen::Matrix2d P0 = Eigen::Vector2d(4.0, 1.0).asDiagonal();
    kf.initialize(x0, P0);
    ukf.initialize(x0, P0);

    std::mt19937 rng(3);
    std::normal_distribution<double> n01(0.0, 1.0);
    for (int k = 0; k < 100; ++k)
    {
        const Eigen::Matrix<double,1,1> z(0.1 * k + n01(rng));
        kf.predict();
        ukf.predict();
        kf.update(z);
        ukf.update(meas, z);
    }
    EXPECT_TRUE(ukf.getState().isApprox(kf.getState(), 1e-9));
    EXPECT_TRUE(ukf.getCovariance().isApprox(kf.getCovariance(), 1e-9));
}

TEST(UnscentedKalmanFilter, MatchesKalmanFilterWithCustomParameters)
{
    const kfl_test::ConstantVelocity cv(0.1, 0.5, 0.25);
    kfl::KalmanFilter<2,1> kf;
    kf.setSystemMatrices(cv.F, cv.Q, cv.H, cv.R);

    kfl::UkfParameters params;
    params.alpha = 0.5;
    params.kappa = 1.0;
    kfl::UnscentedKalmanFilter<2> ukf(std::make_shared<kfl::LinearSystemModel<2>>(cv.F, cv.Q), params);
    const kfl::LinearMeasurementModel<2,1> meas(cv.H, cv.R);

    kf.initialize(Eigen::Vector2d(0, 0), Eigen::Matrix2d::Identity());
    ukf.initialize(Eigen::Vector2d(0, 0), Eigen::Matrix2d::Identity());
    for (int k = 0; k < 50; ++k)
    {
        const Eigen::Matrix<double,1,1> z(std::sin(0.1 * k));
        kf.predict();
        ukf.predict();
        kf.update(z);
        ukf.update(meas, z);
    }
    EXPECT_TRUE(ukf.getState().isApprox(kf.getState(), 1e-9));
    EXPECT_TRUE(ukf.getCovariance().isApprox(kf.getCovariance(), 1e-9));
}

TEST(UnscentedKalmanFilter, RejectsInvalidParameters)
{
    auto model = std::make_shared<DifferentialDriveModel>(0.1, 0.1, 0.1);
    kfl::UkfParameters params;
    params.kappa = -3.0;  // n + lambda = 0
    EXPECT_THROW((kfl::UnscentedKalmanFilter<3,2>(model, params)), std::invalid_argument);
}

TEST(UnscentedKalmanFilter, DiffDriveWithGpsBeatsDeadReckoning)
{
    kfl_test::DiffDriveScenario sc;
    sc.sigma_v = 0.1;
    sc.sigma_omega = 0.05;
    sc.odom_v_scale = 1.03;
    sc.odom_omega_bias = 0.02;
    // Q, modellenmemiş sistematik sapmayı da kapsayacak şekilde büyütüldü (tuning)
    auto model = std::make_shared<DifferentialDriveModel>(sc.dt, sc.sigma_v, 0.4);
    kfl::UnscentedKalmanFilter<3,2> ukf(model);

    const auto res = kfl_test::runDiffDrive(ukf, *model, sc);
    EXPECT_LT(res.pos_rmse, 0.8);
    EXPECT_LT(res.pos_rmse, 0.5 * res.dead_reckoning_rmse);
}

TEST(UnscentedKalmanFilter, DiffDriveWithLandmarksAndHeadingWrap)
{
    kfl_test::DiffDriveScenario sc;
    sc.initial_heading = 3.1;
    sc.use_gps = false;
    sc.use_landmarks = true;
    auto model = std::make_shared<DifferentialDriveModel>(sc.dt, sc.sigma_v, sc.sigma_omega);
    kfl::UnscentedKalmanFilter<3,2> ukf(model);

    const auto res = kfl_test::runDiffDrive(ukf, *model, sc);
    EXPECT_TRUE(res.heading_always_normalized);
    EXPECT_LT(res.pos_rmse, 0.1);
    EXPECT_LT(res.heading_rmse, 0.02);
}

TEST(UnscentedKalmanFilter, GatingRejectsGpsOutliers)
{
    kfl_test::DiffDriveScenario sc;
    sc.gps_outlier_prob = 0.15;
    sc.nis_gate = 9.21;
    auto model = std::make_shared<DifferentialDriveModel>(sc.dt, sc.sigma_v, sc.sigma_omega);
    kfl::UnscentedKalmanFilter<3,2> ukf(model);

    const auto res = kfl_test::runDiffDrive(ukf, *model, sc);
    EXPECT_GT(res.rejected, 0);
    EXPECT_LT(res.pos_rmse, 0.8);
}
