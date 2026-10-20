// Diferansiyel sürüşlü mobil robotta odometri + GPS füzyonu: EKF ve UKF karşılaştırması.
//
// - Odometri (50 Hz): gürültülü ve sistematik hatalı (tekerlek yarıçapı %3, açısal hız sapması)
// - GPS (1 Hz): sigma = 1 m, ölçümlerin %10'u 15 m sapmalı aykırı değer (multipath vb.)
// - Aykırı GPS ölçümleri NIS (ki-kare) testiyle reddedilir
//
// Kullanım: ./example_2d_differential_kinematics [çıktı.csv]

#include <kalman_filter_lib/kalman_filter_lib.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <random>

using kfl::models::DifferentialDriveModel;
using kfl::models::PositionMeasurement;

int main(int argc, char** argv)
{
    const char* out_path = argc > 1 ? argv[1] : "2d_differential_kinematics.csv";

    const double dt = 0.02;           // 50 Hz
    const int steps = 4500;           // 90 s
    const int gps_every = 50;         // 1 Hz
    const double gps_sigma = 1.0;
    const double gps_outlier_prob = 0.1;
    const double nis_gate = 9.21;     // ki-kare, 2 serbestlik derecesi, %99

    // Gerçek odometri hataları (filtre bunları bilmiyor)
    const double sigma_v = 0.1, sigma_omega = 0.05;
    const double v_scale = 1.03, omega_bias = 0.02;

    // Filtre modeli: Q, modellenmemiş sistematik hataları da kapsayacak şekilde ayarlandı
    auto model = std::make_shared<DifferentialDriveModel>(dt, sigma_v, 0.4);
    // Karşılaştırma için: Q sadece gerçek beyaz gürültüye göre ayarlanmış (ayarsız) EKF
    auto naive_model = std::make_shared<DifferentialDriveModel>(dt, sigma_v, sigma_omega);
    const PositionMeasurement gps(gps_sigma);

    kfl::ExtendedKalmanFilter<3,2> ekf(model);
    kfl::UnscentedKalmanFilter<3,2> ukf(model);
    kfl::ExtendedKalmanFilter<3,2> ekf_naive(naive_model);

    const Eigen::Vector3d x0(0.0, 0.0, 0.0);
    const Eigen::Matrix3d P0 = Eigen::Vector3d(0.25, 0.25, 0.01).asDiagonal();
    ekf.initialize(x0, P0);
    ukf.initialize(x0, P0);
    ekf_naive.initialize(x0, P0);

    std::mt19937 rng(2026);
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    std::ofstream csv(out_path);
    csv << "t,true_x,true_y,true_th,dr_x,dr_y,ekf_x,ekf_y,ekf_th,ekf_sx,ekf_sy,ukf_x,ukf_y,ukf_th,"
           "naive_x,naive_y,gps_x,gps_y,gps_rejected\n";

    Eigen::Vector3d truth = x0, dead_reckoning = x0;
    double se_dr = 0.0, se_ekf = 0.0, se_ukf = 0.0, se_naive = 0.0, se_gps = 0.0;
    int n_gps = 0, n_outliers = 0, n_rejected = 0, n_rejected_outliers = 0;
    for (int k = 1; k <= steps; ++k)
    {
        const double t = k * dt;
        // Sekiz (8) şeklinde yörünge
        const Eigen::Vector2d u_true(1.0, 0.35 * std::sin(0.15 * t));
        truth = model->f(truth, u_true);

        const Eigen::Vector2d u_odom(v_scale * u_true(0) + sigma_v * n01(rng),
                                     u_true(1) + omega_bias + sigma_omega * n01(rng));
        dead_reckoning = model->f(dead_reckoning, u_odom);
        ekf.predict(u_odom);
        ukf.predict(u_odom);
        ekf_naive.predict(u_odom);

        double gps_x = std::nan(""), gps_y = std::nan("");
        int rejected = 0;
        if (k % gps_every == 0)
        {
            Eigen::Vector2d z = truth.head<2>() + gps_sigma * Eigen::Vector2d(n01(rng), n01(rng));
            const bool outlier = u01(rng) < gps_outlier_prob;
            if (outlier)
            {
                const double ang = 2.0 * kfl::utils::kPi * u01(rng);
                z += 15.0 * Eigen::Vector2d(std::cos(ang), std::sin(ang));
                ++n_outliers;
            }
            rejected = ekf.update(gps, z, nis_gate) ? 0 : 1;
            ukf.update(gps, z, nis_gate);
            ekf_naive.update(gps, z, nis_gate);

            gps_x = z.x();
            gps_y = z.y();
            se_gps += (z - truth.head<2>()).squaredNorm();
            ++n_gps;
            n_rejected += rejected;
            if (rejected && outlier) ++n_rejected_outliers;
        }

        const auto& xe = ekf.getState();
        const auto& xu = ukf.getState();
        const auto& xn = ekf_naive.getState();
        const auto& Pe = ekf.getCovariance();
        se_dr += (dead_reckoning.head<2>() - truth.head<2>()).squaredNorm();
        se_ekf += (xe.head<2>() - truth.head<2>()).squaredNorm();
        se_ukf += (xu.head<2>() - truth.head<2>()).squaredNorm();
        se_naive += (xn.head<2>() - truth.head<2>()).squaredNorm();

        csv << t << ',' << truth(0) << ',' << truth(1) << ',' << truth(2) << ','
            << dead_reckoning(0) << ',' << dead_reckoning(1) << ','
            << xe(0) << ',' << xe(1) << ',' << xe(2) << ',' << std::sqrt(Pe(0,0)) << ',' << std::sqrt(Pe(1,1)) << ','
            << xu(0) << ',' << xu(1) << ',' << xu(2) << ','
            << xn(0) << ',' << xn(1) << ',' << gps_x << ',' << gps_y << ',' << rejected << '\n';
    }

    std::printf("2D diferansiyel sürüş (odometri + GPS, %d adım)\n", steps);
    std::printf("  Konum RMSE - dead reckoning: %.2f m | ham GPS (aykırılar dahil): %.2f m\n",
                std::sqrt(se_dr / steps), std::sqrt(se_gps / n_gps));
    std::printf("  Konum RMSE - EKF (ayarsız Q): %.2f m | EKF: %.2f m | UKF: %.2f m\n",
                std::sqrt(se_naive / steps), std::sqrt(se_ekf / steps), std::sqrt(se_ukf / steps));
    std::printf("  GPS: %d ölçüm, %d aykırı değer, EKF %d ölçümü reddetti (%d tanesi gerçek aykırı)\n",
                n_gps, n_outliers, n_rejected, n_rejected_outliers);
    std::printf("  CSV: %s\n", out_path);
    return 0;
}
