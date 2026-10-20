#pragma once

#include "../core/types.hpp"
#include "../utils/math_utils.hpp"

namespace kfl
{

/**
 * @brief Doğrusal olmayan ölçüm modeli arayüzü: z = h(x) + v,  v ~ N(0,R)
 *
 * Her sensör kendi ölçüm modeliyle temsil edilir; filtre farklı modellerle sırayla güncellenebilir.
 * H() tanımlanmazsa sayısal türev kullanılır. Ölçümde açı varsa residual() override edilmelidir.
 *
 * @tparam StateDim Durum boyutu
 * @tparam MeasDim  Ölçüm boyutu
 */
template <int StateDim, int MeasDim>
class MeasurementModel
{
    public:
        using State = StateVector<StateDim>;
        using Measurement = MeasurementVector<MeasDim>;
        using Jacobian = Matrix<MeasDim,StateDim>;
        using Covariance = MeasurementCovariance<MeasDim>;

        virtual ~MeasurementModel() = default;

        /// Beklenen ölçüm h(x)
        virtual Measurement h(const State& x) const = 0;

        /// Ölçüm gürültüsü kovaryansı R
        virtual Covariance R() const = 0;

        /// dh/dx Jacobian'ı. Varsayılan: merkezi farklarla sayısal türev
        virtual Jacobian H(const State& x) const
        {
            return utils::numericalJacobian<MeasDim,StateDim>(
                [&](const State& s) { return h(s); },
                [&](const Measurement& a, const Measurement& b) { return residual(a, b); },
                x);
        }

        /// Ölçüm farkı (a - b). Açı içeren ölçümlerde sarmalı fark tanımlanmalı.
        virtual Measurement residual(const Measurement& a, const Measurement& b) const { return a - b; }
};

/**
 * @brief Doğrusal ölçüm modeli: z = H*x + v  (örn. GPS konumu, barometre irtifası)
 */
template <int StateDim, int MeasDim>
class LinearMeasurementModel : public MeasurementModel<StateDim,MeasDim>
{
    public:
        using Base = MeasurementModel<StateDim,MeasDim>;
        using typename Base::State;
        using typename Base::Measurement;
        using typename Base::Jacobian;
        using typename Base::Covariance;

        LinearMeasurementModel(const Jacobian& H, const Covariance& R) : H_(H), R_(R) {}

        Measurement h(const State& x) const override { return H_ * x; }
        Covariance R() const override { return R_; }
        Jacobian H(const State&) const override { return H_; }

        void setR(const Covariance& R) { R_ = R; }

    private:
        Jacobian H_;
        Covariance R_;
};

} // namespace kfl
