#pragma once

#include <kalman_filter_lib/kalman_filter_lib.hpp>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

namespace kfl_test
{

/// Sabit hızlı (constant velocity) 1B model: x = [p, v], beyaz ivme gürültüsü (spektral yoğunluk q)
struct ConstantVelocity
{
    Eigen::Matrix2d F;
    Eigen::Matrix2d Q;
    Eigen::Matrix<double,1,2> H;
    Eigen::Matrix<double,1,1> R;

    ConstantVelocity(double dt, double q, double r)
    {
        F << 1.0, dt,
             0.0, 1.0;
        Q << dt * dt * dt / 3.0, dt * dt / 2.0,
             dt * dt / 2.0,      dt;
        Q *= q;
        H << 1.0, 0.0;
        R << r;
    }
};

/// Çok değişkenli normal dağılımdan örnek: mean + chol(cov) * n
template <int N>
kfl::Matrix<N,1> sampleGaussian(std::mt19937& rng, const kfl::Matrix<N,1>& mean, const kfl::Matrix<N,N>& cov)
{
    std::normal_distribution<double> n01(0.0, 1.0);
    kfl::Matrix<N,1> n;
    for (int i = 0; i < N; ++i) n(i) = n01(rng);
    const kfl::Matrix<N,N> L = cov.llt().matrixL();
    return mean + L * n;
}

struct DiffDriveScenario
{
    int steps = 3000;              // 60 s @ 50 Hz
    double dt = 0.02;
    double sigma_v = 0.05;         // enkoder hız gürültüsü [m/s]
    double sigma_omega = 0.02;     // enkoder açısal hız gürültüsü [rad/s]
    double odom_v_scale = 1.0;     // tekerlek yarıçapı hatası gibi sistematik ölçek hatası
    double odom_omega_bias = 0.0;  // sistematik açısal hız sapması [rad/s]
    double initial_heading = 0.0;
    bool use_gps = true;
    int gps_every = 50;            // 1 Hz
    double gps_sigma = 1.0;
    double gps_outlier_prob = 0.0;
    double gps_outlier_offset = 15.0;
    bool use_landmarks = false;
    int landmark_every = 10;       // 5 Hz
    double nis_gate = kfl::FilterBase<3>::kNoGate;
    unsigned seed = 42;
};

struct DiffDriveResult
{
    double pos_rmse = 0.0;
    double heading_rmse = 0.0;
    double dead_reckoning_rmse = 0.0;
    int rejected = 0;
    bool heading_always_normalized = true;
};

/**
 * @brief Diferansiyel sürüşlü robotu simüle edip verilen filtreyi (EKF veya UKF) çalıştırır.
 * Her iki filtre de aynı arayüzü (predict(u), update(model, z, gate)) sağladığı için şablon kullanılır.
 */
template <typename Filter>
DiffDriveResult runDiffDrive(Filter& filter, kfl::models::DifferentialDriveModel& model,
                             const DiffDriveScenario& sc)
{
    std::mt19937 rng(sc.seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    const kfl::models::PositionMeasurement gps(sc.gps_sigma);
    const std::vector<kfl::models::RangeBearingMeasurement> landmarks = {
        {-6.0, -2.0, 0.1, 0.02}, {6.0, -2.0, 0.1, 0.02}, {-6.0, 10.0, 0.1, 0.02}, {6.0, 10.0, 0.1, 0.02}};

    Eigen::Vector3d truth(0.0, 0.0, sc.initial_heading);
    Eigen::Vector3d dead_reckoning = truth;
    filter.initialize(truth, Eigen::Vector3d(0.25, 0.25, 0.01).asDiagonal());

    DiffDriveResult res;
    double pos_se = 0.0, head_se = 0.0, dr_se = 0.0;
    for (int k = 1; k <= sc.steps; ++k)
    {
        const double t = k * sc.dt;
        const Eigen::Vector2d u_true(1.0, 0.25 + 0.1 * std::sin(0.3 * t));
        truth = model.f(truth, u_true);

        const Eigen::Vector2d u_odom(sc.odom_v_scale * u_true(0) + sc.sigma_v * n01(rng),
                                     u_true(1) + sc.odom_omega_bias + sc.sigma_omega * n01(rng));
        dead_reckoning = model.f(dead_reckoning, u_odom);
        filter.predict(u_odom);

        if (sc.use_gps && k % sc.gps_every == 0)
        {
            Eigen::Vector2d z = truth.head<2>() + sc.gps_sigma * Eigen::Vector2d(n01(rng), n01(rng));
            if (u01(rng) < sc.gps_outlier_prob) z.x() += sc.gps_outlier_offset;
            if (!filter.update(gps, z, sc.nis_gate)) ++res.rejected;
        }
        if (sc.use_landmarks && k % sc.landmark_every == 0)
        {
            for (const auto& lm : landmarks)
            {
                Eigen::Vector2d z = lm.h(truth);
                z(0) += 0.1 * n01(rng);
                z(1) = kfl::utils::normalizeAngle(z(1) + 0.02 * n01(rng));
                filter.update(lm, z, sc.nis_gate);
            }
        }

        const Eigen::Vector3d x = filter.getState();
        if (x(2) < -kfl::utils::kPi || x(2) >= kfl::utils::kPi) res.heading_always_normalized = false;
        pos_se += (x.head<2>() - truth.head<2>()).squaredNorm();
        head_se += std::pow(kfl::utils::normalizeAngle(x(2) - truth(2)), 2);
        dr_se += (dead_reckoning.head<2>() - truth.head<2>()).squaredNorm();
    }
    res.pos_rmse = std::sqrt(pos_se / sc.steps);
    res.heading_rmse = std::sqrt(head_se / sc.steps);
    res.dead_reckoning_rmse = std::sqrt(dr_se / sc.steps);
    return res;
}

} // namespace kfl_test
