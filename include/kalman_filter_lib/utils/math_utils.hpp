#pragma once

#include "../core/types.hpp"

#include <algorithm>
#include <cmath>

namespace kfl
{
namespace utils
{

constexpr double kPi = 3.14159265358979323846;

/**
 * @brief Açıyı [-pi, pi) aralığına sarar.
 */
inline double normalizeAngle(double angle)
{
    angle = std::fmod(angle + kPi, 2.0 * kPi);
    if (angle < 0.0) angle += 2.0 * kPi;
    return angle - kPi;
}

/**
 * @brief Kovaryans matrisini simetrik hale getirir.
 * Kayan nokta hataları zamanla P'nin simetrisini bozabilir; her adımda düzeltmek ucuz ve güvenlidir.
 */
template <typename Derived>
typename Derived::PlainObject symmetrize(const Eigen::MatrixBase<Derived>& M)
{
    return 0.5 * (M + M.transpose());
}

/**
 * @brief Merkezi farklarla sayısal Jacobian: J = df/dx
 * @param f    Türevi alınacak fonksiyon, Matrix<InDim,1> -> Matrix<OutDim,1>
 * @param diff Çıktı uzayındaki fark fonksiyonu (açılar için sarmalı fark kullanılabilir)
 * @param x    Türevin alınacağı nokta
 * @param eps  Göreli adım büyüklüğü
 */
template <int OutDim, int InDim, typename Func, typename Diff>
Matrix<OutDim,InDim> numericalJacobian(const Func& f, const Diff& diff,
                                       const Matrix<InDim,1>& x, double eps = 1e-6)
{
    Matrix<OutDim,InDim> J;
    for (int i = 0; i < InDim; ++i)
    {
        const double h = eps * std::max(1.0, std::abs(x(i)));
        Matrix<InDim,1> x_plus = x;
        Matrix<InDim,1> x_minus = x;
        x_plus(i) += h;
        x_minus(i) -= h;
        J.col(i) = diff(f(x_plus), f(x_minus)) / (2.0 * h);
    }
    return J;
}

/**
 * @brief Standart çıkarma farkı ile sayısal Jacobian.
 */
template <int OutDim, int InDim, typename Func>
Matrix<OutDim,InDim> numericalJacobian(const Func& f, const Matrix<InDim,1>& x, double eps = 1e-6)
{
    return numericalJacobian<OutDim,InDim>(
        f, [](const Matrix<OutDim,1>& a, const Matrix<OutDim,1>& b) { return Matrix<OutDim,1>(a - b); },
        x, eps);
}

} // namespace utils
} // namespace kfl
