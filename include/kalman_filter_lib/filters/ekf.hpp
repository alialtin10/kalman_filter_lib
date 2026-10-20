#pragma once

#include "../core/filter_base.hpp"
#include "../models/measurement_model.hpp"
#include "../models/system_model.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace kfl
{
    /**
     * @brief Genişletilmiş Kalman Filtresi (Extended Kalman Filter)
     *
     * Doğrusal olmayan f() ve h() fonksiyonları, mevcut kestirim etrafında birinci dereceden
     * Taylor açılımı (Jacobian) ile doğrusallaştırılır:
     *
     *   Tahmin:     x = f(x,u)           P = F*P*F^T + Q          (F = df/dx)
     *   Güncelleme: y = z - h(x)         S = H*P*H^T + R          (H = dh/dx)
     *               K = P*H^T*S^-1       x = x + K*y              P = Joseph formu
     *
     * @tparam StateDim Durum boyutu
     * @tparam CtrlDim  Kontrol girdisi boyutu (0: kontrol girdisi yok)
     */
    template <int StateDim, int CtrlDim = 0>
    class ExtendedKalmanFilter : public FilterBase<StateDim>
    {
        public:
            using Base = FilterBase<StateDim>;
            using Base::x_;
            using Base::P_;
            using Model = SystemModel<StateDim,CtrlDim>;

            explicit ExtendedKalmanFilter(std::shared_ptr<const Model> model)
                : model_(std::move(model))
            {
                if (!model_) throw std::invalid_argument("kfl: EKF için sistem modeli boş olamaz");
            }

            // Tahmin Adımı
            void predict()
            {
                // Kontrol girdisi olmadan tahmin
                predict(ControlVector<CtrlDim>::Zero());
            }

            void predict(const ControlVector<CtrlDim>& u)
            {
                this->requireInitialized();

                // Jacobian ve Q, tahmin öncesi durumda (x_{k-1}) hesaplanır
                const typename Model::Jacobian F = model_->F(x_, u);
                const typename Model::Covariance Q = model_->Q(x_, u);

                // x = f(x,u)
                x_ = model_->f(x_, u);
                model_->normalize(x_);

                // P = F*P*F^T + Q
                P_ = utils::symmetrize(F * P_ * F.transpose() + Q);
            }

            /**
             * @brief Güncelleme Adımı - Herhangi bir ölçüm modeliyle durumu düzeltir.
             * @param meas Sensörün ölçüm modeli
             * @param z    Sensör ölçümü
             * @param nis_gate NIS eşiği; aşılırsa ölçüm aykırı değer sayılıp reddedilir
             * @return Ölçüm kabul edildiyse true
             */
            template <int MeasDim>
            bool update(const MeasurementModel<StateDim,MeasDim>& meas,
                        const MeasurementVector<MeasDim>& z,
                        double nis_gate = Base::kNoGate)
            {
                this->requireInitialized();

                // y = z - h(x)  (açılar için modelin residual() fonksiyonu kullanılır)
                const MeasurementVector<MeasDim> y = meas.residual(z, meas.h(x_));
                const bool accepted = this->josephUpdate(y, meas.H(x_), meas.R(), nis_gate);
                model_->normalize(x_);
                return accepted;
            }

            const Model& model() const { return *model_; }

        private:
            std::shared_ptr<const Model> model_;

    }; // class ExtendedKalmanFilter

} // namespace kfl
