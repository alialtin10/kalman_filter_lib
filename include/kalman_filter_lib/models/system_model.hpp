#pragma once

#include "../core/types.hpp"
#include "../utils/math_utils.hpp"

namespace kfl
{

/**
 * @brief Doğrusal olmayan sistem (hareket) modeli arayüzü: x_k = f(x_{k-1}, u_k) + w,  w ~ N(0,Q)
 *
 * Türetilen sınıflar en az f() ve Q()'yu tanımlamalıdır. F() (Jacobian) tanımlanmazsa
 * sayısal türev kullanılır. Durumda açı varsa difference() ve normalize() override edilmelidir.
 *
 * @tparam StateDim Durum boyutu
 * @tparam CtrlDim  Kontrol girdisi boyutu (0: kontrol girdisi yok)
 */
template <int StateDim, int CtrlDim = 0>
class SystemModel
{
    public:
        using State = StateVector<StateDim>;
        using Control = ControlVector<CtrlDim>;
        using Jacobian = Matrix<StateDim,StateDim>;
        using Covariance = StateCovariance<StateDim>;

        virtual ~SystemModel() = default;

        /// Durum geçiş fonksiyonu f(x,u)
        virtual State f(const State& x, const Control& u) const = 0;

        /// Süreç gürültüsü kovaryansı Q (duruma ve girdiye bağlı olabilir)
        virtual Covariance Q(const State& x, const Control& u) const = 0;

        /// df/dx Jacobian'ı. Varsayılan: merkezi farklarla sayısal türev
        virtual Jacobian F(const State& x, const Control& u) const
        {
            return utils::numericalJacobian<StateDim,StateDim>(
                [&](const State& s) { return f(s, u); },
                [&](const State& a, const State& b) { return difference(a, b); },
                x);
        }

        /// İki durum arasındaki fark (a - b). Açı içeren durumlar için sarmalı fark tanımlanmalı.
        virtual State difference(const State& a, const State& b) const { return a - b; }

        /// Durumu geçerli aralığa getirir (örn. açıyı [-pi,pi) aralığına sarmak)
        virtual void normalize(State& x) const { (void)x; }
};

/**
 * @brief Doğrusal sistem modeli: x_k = F*x_{k-1} + B*u_k + w
 * EKF/UKF'yi doğrusal sistemlerde kullanmak veya KF ile karşılaştırmak için.
 */
template <int StateDim, int CtrlDim = 0>
class LinearSystemModel : public SystemModel<StateDim,CtrlDim>
{
    public:
        using Base = SystemModel<StateDim,CtrlDim>;
        using typename Base::State;
        using typename Base::Control;
        using typename Base::Jacobian;
        using typename Base::Covariance;
        using ControlMatrix = Matrix<StateDim,CtrlDim>;

        LinearSystemModel(const Jacobian& F, const Covariance& Q,
                          const ControlMatrix& B = ControlMatrix::Zero())
            : F_(F), Q_(Q), B_(B) {}

        State f(const State& x, const Control& u) const override { return F_ * x + B_ * u; }
        Covariance Q(const State&, const Control&) const override { return Q_; }
        Jacobian F(const State&, const Control&) const override { return F_; }

    private:
        Jacobian F_;
        Covariance Q_;
        ControlMatrix B_;
};

} // namespace kfl
