// Basit örnek: Gürültülü konum sensörüyle 1B hareket eden robotun konum ve hızını
// sabit hız (constant velocity) modelli standart Kalman Filtresi ile kestirmek.
//
// Kullanım: ./example_1d_robot_tracking [çıktı.csv]

#include <kalman_filter_lib/kalman_filter_lib.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>

int main(int argc, char** argv)
{
    const char* out_path = argc > 1 ? argv[1] : "1d_robot_tracking.csv";

    const double dt = 0.1;          // 10 Hz
    const int steps = 300;          // 30 s
    const double sigma_meas = 0.8;  // konum sensörü gürültüsü [m]
    const double q = 0.3;           // ivme gürültüsü spektral yoğunluğu (modelin bilmediği ivmeyi temsil eder)

    // Durum: x = [konum, hız]
    Eigen::Matrix2d F;
    F << 1.0, dt,
         0.0, 1.0;
    Eigen::Matrix2d Q;
    Q << dt * dt * dt / 3.0, dt * dt / 2.0,
         dt * dt / 2.0,      dt;
    Q *= q;
    const Eigen::Matrix<double,1,2> H(1.0, 0.0);
    const Eigen::Matrix<double,1,1> R(sigma_meas * sigma_meas);

    kfl::KalmanFilter<2,1> kf;
    kf.setSystemMatrices(F, Q, H, R);
    kf.initialize(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0).asDiagonal());

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, sigma_meas);

    std::ofstream csv(out_path);
    csv << "t,true_pos,true_vel,meas_pos,est_pos,est_vel,std_pos\n";

    double pos = 0.0, vel = 0.5;
    double se_meas = 0.0, se_est = 0.0, se_vel = 0.0;
    for (int k = 1; k <= steps; ++k)
    {
        const double t = k * dt;
        // Gerçek hareket: robot sinüzoidal ivmeyle hızlanıp yavaşlıyor (filtre bunu bilmiyor)
        const double a = 0.5 * std::sin(0.4 * t);
        pos += vel * dt + 0.5 * a * dt * dt;
        vel += a * dt;

        const double z = pos + noise(rng);

        kf.predict();
        kf.update(Eigen::Matrix<double,1,1>(z));

        const auto& x = kf.getState();
        const double std_pos = std::sqrt(kf.getCovariance()(0,0));
        csv << t << ',' << pos << ',' << vel << ',' << z << ',' << x(0) << ',' << x(1) << ',' << std_pos << '\n';

        se_meas += (z - pos) * (z - pos);
        se_est += (x(0) - pos) * (x(0) - pos);
        se_vel += (x(1) - vel) * (x(1) - vel);
    }

    std::printf("1D robot tracking (KF, %d adım)\n", steps);
    std::printf("  Konum RMSE  - ham sensör: %.3f m | KF: %.3f m\n", std::sqrt(se_meas / steps), std::sqrt(se_est / steps));
    std::printf("  Hız RMSE    - KF: %.3f m/s (hız doğrudan ölçülmüyor)\n", std::sqrt(se_vel / steps));
    std::printf("  CSV: %s\n", out_path);
    return 0;
}
