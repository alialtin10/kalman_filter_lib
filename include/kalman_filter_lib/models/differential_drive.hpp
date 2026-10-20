#pragma once

#include "measurement_model.hpp"
#include "system_model.hpp"

#include <cmath>

namespace kfl
{
namespace models
{

/**
 * @brief Diferansiyel sürüşlü (veya skid-steer) mobil robotun kinematik modeli.
 *
 * Durum:  x = [px, py, theta]^T
 * Girdi:  u = [v, omega]^T  (enkoder/odometri hızları)
 *
 * Orta nokta (midpoint) integrasyonu:
 *   phi   = theta + omega*dt/2
 *   px'   = px + v*dt*cos(phi)
 *   py'   = py + v*dt*sin(phi)
 *   theta'= theta + omega*dt
 *
 * Süreç gürültüsü, girdi gürültüsünün durum uzayına taşınmasıyla elde edilir: Q = G*M*G^T,
 * G = df/du, M = diag(sigma_v^2, sigma_omega^2).
 */
class DifferentialDriveModel : public SystemModel<3,2>
{
    public:
        DifferentialDriveModel(double dt, double sigma_v, double sigma_omega)
            : dt_(dt), sigma_v_(sigma_v), sigma_omega_(sigma_omega) {}

        void setDt(double dt) { dt_ = dt; }
        double dt() const { return dt_; }

        State f(const State& x, const Control& u) const override
        {
            const double v = u(0), w = u(1);
            const double phi = x(2) + 0.5 * w * dt_;
            State out;
            out << x(0) + v * dt_ * std::cos(phi),
                   x(1) + v * dt_ * std::sin(phi),
                   utils::normalizeAngle(x(2) + w * dt_);
            return out;
        }

        Jacobian F(const State& x, const Control& u) const override
        {
            const double v = u(0);
            const double phi = x(2) + 0.5 * u(1) * dt_;
            Jacobian F = Jacobian::Identity();
            F(0,2) = -v * dt_ * std::sin(phi);
            F(1,2) =  v * dt_ * std::cos(phi);
            return F;
        }

        Covariance Q(const State& x, const Control& u) const override
        {
            const double v = u(0);
            const double phi = x(2) + 0.5 * u(1) * dt_;
            Matrix<3,2> G;
            G << dt_ * std::cos(phi), -0.5 * v * dt_ * dt_ * std::sin(phi),
                 dt_ * std::sin(phi),  0.5 * v * dt_ * dt_ * std::cos(phi),
                 0.0,                  dt_;
            const Matrix<2,2> M = Eigen::Vector2d(sigma_v_ * sigma_v_, sigma_omega_ * sigma_omega_).asDiagonal();
            return G * M * G.transpose();
        }

        State difference(const State& a, const State& b) const override
        {
            State d = a - b;
            d(2) = utils::normalizeAngle(d(2));
            return d;
        }

        void normalize(State& x) const override { x(2) = utils::normalizeAngle(x(2)); }

    private:
        double dt_;
        double sigma_v_;
        double sigma_omega_;
};

/**
 * @brief GPS / RTK benzeri mutlak konum ölçümü: z = [px, py]^T
 */
class PositionMeasurement : public MeasurementModel<3,2>
{
    public:
        explicit PositionMeasurement(double sigma) : sigma_(sigma) {}

        Measurement h(const State& x) const override { return x.head<2>(); }

        Jacobian H(const State&) const override
        {
            Jacobian H = Jacobian::Zero();
            H(0,0) = 1.0;
            H(1,1) = 1.0;
            return H;
        }

        Covariance R() const override { return Covariance::Identity() * sigma_ * sigma_; }

    private:
        double sigma_;
};

/**
 * @brief Bilinen bir landmark'a (işaret noktası) mesafe ve yön ölçümü: z = [r, b]^T
 * LiDAR/kamera ile reflektör, ArUco vb. tespiti için tipik doğrusal olmayan ölçüm modeli.
 *
 *   r = sqrt(dx^2 + dy^2)
 *   b = atan2(dy, dx) - theta
 */
class RangeBearingMeasurement : public MeasurementModel<3,2>
{
    public:
        RangeBearingMeasurement(double landmark_x, double landmark_y, double sigma_range, double sigma_bearing)
            : lx_(landmark_x), ly_(landmark_y), sigma_r_(sigma_range), sigma_b_(sigma_bearing) {}

        Measurement h(const State& x) const override
        {
            const double dx = lx_ - x(0), dy = ly_ - x(1);
            return Measurement(std::hypot(dx, dy), utils::normalizeAngle(std::atan2(dy, dx) - x(2)));
        }

        Jacobian H(const State& x) const override
        {
            const double dx = lx_ - x(0), dy = ly_ - x(1);
            const double q = dx * dx + dy * dy;
            const double r = std::sqrt(q);
            Jacobian H;
            H << -dx / r, -dy / r,  0.0,
                  dy / q, -dx / q, -1.0;
            return H;
        }

        Covariance R() const override
        {
            return Eigen::Vector2d(sigma_r_ * sigma_r_, sigma_b_ * sigma_b_).asDiagonal();
        }

        Measurement residual(const Measurement& a, const Measurement& b) const override
        {
            Measurement d = a - b;
            d(1) = utils::normalizeAngle(d(1));
            return d;
        }

        double landmarkX() const { return lx_; }
        double landmarkY() const { return ly_; }

    private:
        double lx_, ly_;
        double sigma_r_, sigma_b_;
};

} // namespace models
} // namespace kfl
