// İHA irtifa kestirimi: IMU (dikey ivme) + barometre füzyonu, ivmeölçer sapması (bias) kestirimi ile.
//
// Durum:  x = [h, vz, b]^T   (irtifa, dikey hız, ivmeölçer sapması)
// Girdi:  u = a_m            (yerçekimi çıkarılmış dikey ivme ölçümü, 100 Hz)
// Ölçüm:  z = h + v          (barometre, 20 Hz)
//
//   h'  = h + vz*dt + (a_m - b)*dt^2/2
//   vz' = vz + (a_m - b)*dt
//   b'  = b                  (yavaş rastgele yürüyüş)
//
// IMU tek başına sapma yüzünden hızla ıraksar, barometre ise gürültülüdür. KF ikisini birleştirir
// ve sapmayı barometre üzerinden gözlemleyerek kestirir.
//
// Kullanım: ./example_altitude_estimation [çıktı.csv]

#include <kalman_filter_lib/kalman_filter_lib.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>

int main(int argc, char** argv)
{
    const char* out_path = argc > 1 ? argv[1] : "altitude_estimation.csv";

    const double dt = 0.01;            // IMU 100 Hz
    const int steps = 6000;            // 60 s
    const int baro_every = 5;          // barometre 20 Hz
    const double sigma_acc = 0.05;     // ivmeölçer gürültüsü [m/s^2]
    const double true_bias = 0.25;     // ivmeölçer sapması [m/s^2]
    const double sigma_bias_rw = 1e-3; // sapma rastgele yürüyüşü [m/s^2/sqrt(s)]
    const double sigma_baro = 0.4;     // barometre gürültüsü [m]

    Eigen::Matrix3d F;
    F << 1.0, dt,  -0.5 * dt * dt,
         0.0, 1.0, -dt,
         0.0, 0.0, 1.0;
    const Eigen::Vector3d B(0.5 * dt * dt, dt, 0.0);

    // Q: ivme gürültüsünün durum uzayına etkisi (B*sigma^2*B^T) + sapmanın rastgele yürüyüşü
    Eigen::Matrix3d Q = B * B.transpose() * sigma_acc * sigma_acc;
    Q(2,2) += sigma_bias_rw * sigma_bias_rw * dt;

    const Eigen::Matrix<double,1,3> H(1.0, 0.0, 0.0);
    const Eigen::Matrix<double,1,1> R(sigma_baro * sigma_baro);

    kfl::KalmanFilter<3,1,1> kf;
    kf.setSystemMatrices(F, Q, H, R, B);
    kf.initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.1, 0.5 * 0.5).asDiagonal());

    std::mt19937 rng(7);
    std::normal_distribution<double> n01(0.0, 1.0);

    std::ofstream csv(out_path);
    csv << "t,true_h,true_v,baro,imu_only_h,est_h,est_v,est_bias,std_h,std_bias,true_bias\n";

    // Gerçek uçuş profili: 0 -> 20 m tırmanış, iniş, tekrar tırmanış (yumuşak sinüzoidal)
    const double w = 2.0 * kfl::utils::kPi / 40.0;
    auto true_h = [&](double t) { return 10.0 * (1.0 - std::cos(w * t)); };
    auto true_v = [&](double t) { return 10.0 * w * std::sin(w * t); };
    auto true_a = [&](double t) { return 10.0 * w * w * std::cos(w * t); };

    double imu_h = 0.0, imu_v = 0.0;  // Sadece IMU entegrasyonu (karşılaştırma için)
    double baro = std::nan("");
    double se_baro = 0.0, se_est = 0.0, se_v = 0.0;
    int n_baro = 0;
    for (int k = 1; k <= steps; ++k)
    {
        const double t = k * dt;
        const double a_m = true_a(t - dt) + true_bias + sigma_acc * n01(rng);

        imu_h += imu_v * dt + 0.5 * a_m * dt * dt;
        imu_v += a_m * dt;

        kf.predict(Eigen::Matrix<double,1,1>(a_m));

        const bool has_baro = (k % baro_every == 0);
        if (has_baro)
        {
            baro = true_h(t) + sigma_baro * n01(rng);
            kf.update(Eigen::Matrix<double,1,1>(baro));
            se_baro += std::pow(baro - true_h(t), 2);
            ++n_baro;
        }

        const auto& x = kf.getState();
        const auto& P = kf.getCovariance();
        se_est += std::pow(x(0) - true_h(t), 2);
        se_v += std::pow(x(1) - true_v(t), 2);

        csv << t << ',' << true_h(t) << ',' << true_v(t) << ',' << (has_baro ? baro : std::nan("")) << ','
            << imu_h << ',' << x(0) << ',' << x(1) << ',' << x(2) << ',' << std::sqrt(P(0,0)) << ','
            << std::sqrt(P(2,2)) << ',' << true_bias << '\n';
    }

    std::printf("İHA irtifa kestirimi (KF: IMU + barometre, %d adım)\n", steps);
    std::printf("  İrtifa RMSE - barometre: %.3f m | KF: %.3f m\n", std::sqrt(se_baro / n_baro), std::sqrt(se_est / steps));
    std::printf("  Dikey hız RMSE - KF: %.3f m/s\n", std::sqrt(se_v / steps));
    std::printf("  Sadece IMU ile 60 s sonundaki irtifa hatası: %.1f m\n", imu_h - true_h(steps * dt));
    std::printf("  Kestirilen ivmeölçer sapması: %.3f (gerçek: %.3f) m/s^2\n", kf.getState()(2), true_bias);
    std::printf("  CSV: %s\n", out_path);
    return 0;
}
