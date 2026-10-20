#pragma once

#include "../core/filter_base.hpp"
#include<Eigen/Dense>

namespace kfl
{
    /**
     * @brief Standart (doğrusal) Kalman Filtresi
     *
     * x_k = F*x_{k-1} + B*u_k + w,   w ~ N(0,Q)
     * z_k = H*x_k + v,               v ~ N(0,R)
     *
     * @tparam StateDim Durum boyutu
     * @tparam MeasDim  Ölçüm boyutu
     * @tparam CtrlDim  Kontrol girdisi boyutu (0: kontrol girdisi yok)
     */
    template <int StateDim, int MeasDim, int CtrlDim =0>
    class KalmanFilter : public FilterBase<StateDim>
    {
        public:
            // Üst  Sınıf x_ ve P_ üyelerini doğrudan kullanabilmek için
            using Base = FilterBase<StateDim>;
            using Base::x_;
            using Base::P_;

            // ilgili matris tiplerini tanımlayalım
            using StateMatrix = Eigen::Matrix<double,StateDim,StateDim>;
            using ControlMatrix = Eigen::Matrix<double,StateDim,CtrlDim>;
            using MeasurementMatrix = Eigen::Matrix<double,MeasDim,StateDim>;
            using MeasurementCovarianceMatrix = Eigen::Matrix<double,MeasDim,MeasDim>;

            KalmanFilter() = default;
            ~KalmanFilter() override = default;

            // --- Functions ---
            /**
             *@brief Sistemin matrislerini ayarlar (F,B,H,Q,R).
             * dt değişkense her predict() öncesinde yeniden çağrılabilir.
             */
            void setSystemMatrices( const StateMatrix& F,
                                    const StateMatrix& Q,
                                    const MeasurementMatrix& H,
                                    const MeasurementCovarianceMatrix& R,
                                    const ControlMatrix& B = ControlMatrix::Zero())
            {
                F_ = F;
                Q_ = Q;
                H_ = H;
                R_ = R;
                B_ = B;
                has_model_ = true;
            }

            // Tahmin Adımı
            void predict()
            {
                // Kontrol girdisi olmadan tahmin
                predict(ControlVector<CtrlDim>::Zero());
            }

            void predict(const ControlVector<CtrlDim>& u)
            {
                requireReady();

                // x = F*x + B*u
                x_ = F_ * x_ + B_ * u;

                // P = F*P*F^T + Q
                P_ = utils::symmetrize(F_ * P_ * F_.transpose() + Q_);
            }

            /**
             * @brief Güncelleme Adımı - Durumu sensör verisiyle düzeltir.
             * @param z sensör ölçüm vektörü
             * @param nis_gate NIS eşiği; aşılırsa ölçüm aykırı değer sayılıp reddedilir
             * @return Ölçüm kabul edildiyse true
             */
            bool update(const MeasurementVector<MeasDim>& z, double nis_gate = Base::kNoGate)
            {
                requireReady();

                // y = z - H*x
                const MeasurementVector<MeasDim> y = z - H_ * x_;
                return this->josephUpdate(y, H_, R_, nis_gate);
            }

        private:
            void requireReady() const
            {
                this->requireInitialized();
                if (!has_model_)
                    throw std::logic_error("kfl: setSystemMatrices() çağrılmadan KalmanFilter kullanıldı");
            }

            StateMatrix F_; //Sistem / Durum Geçiş Matriksi
            ControlMatrix B_; // Kontrol Matriksi
            MeasurementMatrix H_; // Ölçüm Matriksi
            StateMatrix Q_; // Süreç Gürültüsü (Process Noise) Matrixi
            MeasurementCovarianceMatrix R_; // Ölçüm Gürültüsü (Measurement Noise) Matrixi
            bool has_model_ = false;

    }; // class KalmanFilter


} // namespace kfl
