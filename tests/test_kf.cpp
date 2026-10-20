#include "test_helpers.hpp"

#include <gtest/gtest.h>

using kfl_test::ConstantVelocity;

TEST(KalmanFilter, ThrowsWhenUsedBeforeInitialize)
{
    kfl::KalmanFilter<1,1> kf;
    kf.setSystemMatrices(Eigen::Matrix<double,1,1>(1.0), Eigen::Matrix<double,1,1>(0.0),
                         Eigen::Matrix<double,1,1>(1.0), Eigen::Matrix<double,1,1>(1.0));
    EXPECT_THROW(kf.predict(), std::logic_error);
    EXPECT_THROW(kf.update(Eigen::Matrix<double,1,1>(0.0)), std::logic_error);
}

TEST(KalmanFilter, ThrowsWhenUsedBeforeSystemMatrices)
{
    kfl::KalmanFilter<1,1> kf;
    kf.initialize(Eigen::Matrix<double,1,1>(0.0), Eigen::Matrix<double,1,1>(1.0));
    EXPECT_THROW(kf.predict(), std::logic_error);
}

// Sabit bir değerin tekrarlı ölçümü: kovaryans analitik olarak 1/(1/P0 + N/R) olmalı
TEST(KalmanFilter, ScalarConstantMatchesAnalyticPosterior)
{
    const double P0 = 100.0, R = 1.0, truth = 5.0;
    kfl::KalmanFilter<1,1> kf;
    kf.setSystemMatrices(Eigen::Matrix<double,1,1>(1.0), Eigen::Matrix<double,1,1>(0.0),
                         Eigen::Matrix<double,1,1>(1.0), Eigen::Matrix<double,1,1>(R));
    kf.initialize(Eigen::Matrix<double,1,1>(0.0), Eigen::Matrix<double,1,1>(P0));

    std::mt19937 rng(1);
    std::normal_distribution<double> noise(0.0, std::sqrt(R));
    const int N = 1000;
    for (int i = 0; i < N; ++i)
    {
        kf.predict();
        kf.update(Eigen::Matrix<double,1,1>(truth + noise(rng)));
    }
    EXPECT_NEAR(kf.getState()(0), truth, 0.1);
    EXPECT_NEAR(kf.getCovariance()(0,0), 1.0 / (1.0 / P0 + N / R), 1e-12);
}

// Rastgele yürüyüş modelinde kovaryans, ayrık Riccati denkleminin sabit durum çözümüne yakınsamalı
TEST(KalmanFilter, ConvergesToSteadyStateRiccatiSolution)
{
    const double q = 0.3, r = 2.0;
    kfl::KalmanFilter<1,1> kf;
    kf.setSystemMatrices(Eigen::Matrix<double,1,1>(1.0), Eigen::Matrix<double,1,1>(q),
                         Eigen::Matrix<double,1,1>(1.0), Eigen::Matrix<double,1,1>(r));
    kf.initialize(Eigen::Matrix<double,1,1>(0.0), Eigen::Matrix<double,1,1>(50.0));
    for (int i = 0; i < 200; ++i)
    {
        kf.predict();
        kf.update(Eigen::Matrix<double,1,1>(0.0));
    }
    // P- = (q + sqrt(q^2 + 4qr)) / 2,  P+ = P- * r / (P- + r)
    const double P_prior = 0.5 * (q + std::sqrt(q * q + 4.0 * q * r));
    const double P_post = P_prior * r / (P_prior + r);
    EXPECT_NEAR(kf.getCovariance()(0,0), P_post, 1e-10);
}

TEST(KalmanFilter, ControlInputIntegratesKinematicsExactly)
{
    const double dt = 0.1, a = 2.0;
    Eigen::Matrix2d F;
    F << 1, dt, 0, 1;
    const Eigen::Vector2d B(0.5 * dt * dt, dt);

    kfl::KalmanFilter<2,1,1> kf;
    kf.setSystemMatrices(F, Eigen::Matrix2d::Zero(), Eigen::Matrix<double,1,2>(1, 0),
                         Eigen::Matrix<double,1,1>(1.0), B);
    kf.initialize(Eigen::Vector2d(0, 0), Eigen::Matrix2d::Identity());
    for (int i = 0; i < 50; ++i) kf.predict(Eigen::Matrix<double,1,1>(a));

    const double t = 50 * dt;
    EXPECT_NEAR(kf.getState()(0), 0.5 * a * t * t, 1e-9);
    EXPECT_NEAR(kf.getState()(1), a * t, 1e-9);
}

// Monte Carlo tutarlılık testi: NEES ortalaması ~ durum boyutu (2), NIS ortalaması ~ ölçüm boyutu (1)
TEST(KalmanFilter, ConstantVelocityFilterIsConsistent)
{
    const ConstantVelocity cv(0.1, 0.5, 0.25);
    std::mt19937 rng(7);

    const int runs = 50, steps = 200;
    double nees_sum = 0.0, nis_sum = 0.0, se_meas = 0.0, se_est = 0.0;
    for (int run = 0; run < runs; ++run)
    {
        const Eigen::Vector2d x0(0.0, 1.0);
        const Eigen::Matrix2d P0 = Eigen::Matrix2d::Identity();
        Eigen::Vector2d truth = kfl_test::sampleGaussian<2>(rng, x0, P0);

        kfl::KalmanFilter<2,1> kf;
        kf.setSystemMatrices(cv.F, cv.Q, cv.H, cv.R);
        kf.initialize(x0, P0);
        for (int k = 0; k < steps; ++k)
        {
            truth = kfl_test::sampleGaussian<2>(rng, Eigen::Vector2d(cv.F * truth), cv.Q);
            const auto z = kfl_test::sampleGaussian<1>(rng, cv.H * truth, cv.R);
            kf.predict();
            kf.update(z);

            const Eigen::Vector2d e = truth - kf.getState();
            nees_sum += e.dot(kf.getCovariance().ldlt().solve(e));
            nis_sum += kf.getLastNIS();
            se_meas += std::pow(z(0) - truth(0), 2);
            se_est += e(0) * e(0);
        }
    }
    const double n = runs * steps;
    EXPECT_NEAR(nees_sum / n, 2.0, 0.3);
    EXPECT_NEAR(nis_sum / n, 1.0, 0.15);
    EXPECT_LT(se_est, 0.6 * se_meas);  // filtre ham ölçümden belirgin şekilde daha iyi olmalı
}

TEST(KalmanFilter, NisGateRejectsOutlierAndKeepsState)
{
    const ConstantVelocity cv(0.1, 0.1, 0.25);
    kfl::KalmanFilter<2,1> kf;
    kf.setSystemMatrices(cv.F, cv.Q, cv.H, cv.R);
    kf.initialize(Eigen::Vector2d(0, 0), Eigen::Matrix2d::Identity());
    for (int i = 0; i < 100; ++i)
    {
        kf.predict();
        ASSERT_TRUE(kf.update(Eigen::Matrix<double,1,1>(0.0), 9.0));
    }
    kf.predict();
    const Eigen::Vector2d before = kf.getState();
    const Eigen::Matrix2d P_before = kf.getCovariance();

    EXPECT_FALSE(kf.update(Eigen::Matrix<double,1,1>(50.0), 9.0));  // 1 DoF, ~%99.7
    EXPECT_GT(kf.getLastNIS(), 9.0);
    EXPECT_TRUE(kf.getState().isApprox(before));
    EXPECT_TRUE(kf.getCovariance().isApprox(P_before));
}

TEST(KalmanFilter, CovarianceStaysSymmetricPositiveDefinite)
{
    const ConstantVelocity cv(0.01, 1e-6, 1e-4);
    kfl::KalmanFilter<2,1> kf;
    kf.setSystemMatrices(cv.F, cv.Q, cv.H, cv.R);
    kf.initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(1e6, 1e6).asDiagonal());
    for (int i = 0; i < 10000; ++i)
    {
        kf.predict();
        kf.update(Eigen::Matrix<double,1,1>(0.001 * i));
    }
    const Eigen::Matrix2d P = kf.getCovariance();
    EXPECT_EQ(P, P.transpose());
    EXPECT_GT(Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(P).eigenvalues().minCoeff(), 0.0);
}
