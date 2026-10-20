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
     * @brief Sigma noktası (Scaled Unscented Transform) parametreleri.
     *
     * lambda = alpha^2 * (n + kappa) - n
     * Varsayılan (alpha=1, beta=2, kappa=0) sigma noktalarını sqrt(n)*sigma uzaklığa yerleştirir ve
     * tüm ağırlıkları pozitif tutar; küçük alpha değerleri (örn. 1e-3) büyük negatif merkez ağırlığı
     * üreterek sayısal sorunlara yol açabilir.
     */
    struct UkfParameters
    {
        double alpha = 1.0;
        double beta = 2.0;   // Gauss dağılımı için optimum
        double kappa = 0.0;
    };

    /**
     * @brief Unscented Kalman Filtresi
     *
     * Jacobian hesaplamak yerine 2n+1 sigma noktasını doğrusal olmayan fonksiyonlardan geçirip
     * ortalama ve kovaryansı bu noktalardan yeniden hesaplar. Güçlü doğrusal olmayan sistemlerde
     * EKF'ye göre daha doğru, doğrusal sistemlerde ise KF ile birebir aynı sonucu verir.
     *
     * @tparam StateDim Durum boyutu
     * @tparam CtrlDim  Kontrol girdisi boyutu (0: kontrol girdisi yok)
     */
    template <int StateDim, int CtrlDim = 0>
    class UnscentedKalmanFilter : public FilterBase<StateDim>
    {
        public:
            using Base = FilterBase<StateDim>;
            using Base::x_;
            using Base::P_;
            using Model = SystemModel<StateDim,CtrlDim>;
            using State = StateVector<StateDim>;
            using Covariance = StateCovariance<StateDim>;

            static constexpr int kSigmaCount = 2 * StateDim + 1;
            using SigmaPoints = Matrix<StateDim,kSigmaCount>;
            using Weights = Matrix<kSigmaCount,1>;

            explicit UnscentedKalmanFilter(std::shared_ptr<const Model> model,
                                           const UkfParameters& params = UkfParameters())
                : model_(std::move(model))
            {
                if (!model_) throw std::invalid_argument("kfl: UKF için sistem modeli boş olamaz");

                const double n = StateDim;
                const double lambda = params.alpha * params.alpha * (n + params.kappa) - n;
                scale_ = n + lambda;
                if (scale_ <= 0.0) throw std::invalid_argument("kfl: UKF parametreleri n + lambda > 0 sağlamalı");

                wm_.setConstant(0.5 / scale_);
                wc_.setConstant(0.5 / scale_);
                wm_(0) = lambda / scale_;
                wc_(0) = lambda / scale_ + (1.0 - params.alpha * params.alpha + params.beta);
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

                const Covariance Q = model_->Q(x_, u);

                // Sigma noktalarını f() üzerinden geçir
                const SigmaPoints X = sigmaPoints();
                SigmaPoints Y;
                for (int i = 0; i < kSigmaCount; ++i)
                {
                    State yi = model_->f(X.col(i), u);
                    model_->normalize(yi);
                    Y.col(i) = yi;
                }

                // Ağırlıklı ortalama ve kovaryans
                x_ = stateMean(Y);
                Covariance P = Q;
                for (int i = 0; i < kSigmaCount; ++i)
                {
                    const State d = model_->difference(Y.col(i), x_);
                    P += wc_(i) * d * d.transpose();
                }
                P_ = utils::symmetrize(P);
            }

            /**
             * @brief Güncelleme Adımı - Herhangi bir ölçüm modeliyle durumu düzeltir.
             * @param meas Sensörün ölçüm modeli (Jacobian kullanılmaz, sadece h() ve R())
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
                using Meas = MeasurementVector<MeasDim>;
                using MeasCov = MeasurementCovariance<MeasDim>;

                // Sigma noktalarını h() üzerinden geçir
                const SigmaPoints X = sigmaPoints();
                Matrix<MeasDim,kSigmaCount> Z;
                for (int i = 0; i < kSigmaCount; ++i) Z.col(i) = meas.h(X.col(i));

                // Beklenen ölçüm: z0 + sum(wm_i * (Z_i - z0))  -> açılar için sarmalı fark ile güvenli ortalama
                const Meas z0 = Z.col(0);
                Meas offset = Meas::Zero();
                for (int i = 0; i < kSigmaCount; ++i) offset += wm_(i) * meas.residual(Z.col(i), z0);
                const Meas z_pred = z0 + offset;

                // İnovasyon kovaryansı S ve çapraz kovaryans Pxz
                MeasCov S = meas.R();
                Matrix<StateDim,MeasDim> Pxz = Matrix<StateDim,MeasDim>::Zero();
                for (int i = 0; i < kSigmaCount; ++i)
                {
                    const Meas dz = meas.residual(Z.col(i), z_pred);
                    const State dx = model_->difference(X.col(i), x_);
                    S += wc_(i) * dz * dz.transpose();
                    Pxz += wc_(i) * dx * dz.transpose();
                }
                S = utils::symmetrize(S);

                const Eigen::LLT<MeasCov> llt(S);
                if (llt.info() != Eigen::Success)
                    throw std::runtime_error("kfl: inovasyon kovaryansı S pozitif tanımlı değil");

                const Meas y = meas.residual(z, z_pred);
                this->last_nis_ = y.dot(llt.solve(y));
                if (this->last_nis_ > nis_gate) return false;

                // K = Pxz*S^-1  <=>  K^T = S^-1*Pxz^T
                const Matrix<StateDim,MeasDim> K = llt.solve(Pxz.transpose()).transpose();
                x_ += K * y;
                model_->normalize(x_);
                P_ = utils::symmetrize(P_ - K * S * K.transpose());
                return true;
            }

            const Model& model() const { return *model_; }

        private:
            /// Mevcut (x,P) etrafında 2n+1 sigma noktası üretir: x, x +- sqrt((n+lambda)P)
            SigmaPoints sigmaPoints() const
            {
                const Eigen::LLT<Covariance> llt(scale_ * P_);
                if (llt.info() != Eigen::Success)
                    throw std::runtime_error("kfl: kovaryans P pozitif tanımlı değil (Cholesky başarısız)");
                const Covariance L = llt.matrixL();

                SigmaPoints X;
                X.col(0) = x_;
                for (int i = 0; i < StateDim; ++i)
                {
                    State plus = x_ + L.col(i);
                    State minus = x_ - L.col(i);
                    model_->normalize(plus);
                    model_->normalize(minus);
                    X.col(1 + i) = plus;
                    X.col(1 + StateDim + i) = minus;
                }
                return X;
            }

            /// Sigma noktalarının ağırlıklı ortalaması (açılar için referans noktasına göre fark ile)
            State stateMean(const SigmaPoints& Y) const
            {
                const State ref = Y.col(0);
                State offset = State::Zero();
                for (int i = 0; i < kSigmaCount; ++i) offset += wm_(i) * model_->difference(Y.col(i), ref);
                State mean = ref + offset;
                model_->normalize(mean);
                return mean;
            }

            std::shared_ptr<const Model> model_;
            double scale_ = 0.0;   // n + lambda
            Weights wm_;           // Ortalama ağırlıkları
            Weights wc_;           // Kovaryans ağırlıkları

    }; // class UnscentedKalmanFilter

} // namespace kfl
