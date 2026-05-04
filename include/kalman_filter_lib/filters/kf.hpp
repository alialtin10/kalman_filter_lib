#pragma once 

#include "../core/filter_base.hpp"
#include<Eigen/Dense>

namespace kfl
{
    /**
     * @brief Standart Kalman Filtresi (Standart Sistemler için)
     */
    template <int StateDim, int MeasDim, int CtrlDim =0>
    class KalmanFilter : public FilterBase<StateDim,MeasDim,CtrlDim>
    {
        public:
            // Üst  Sınıf x_ ve P_ üyerlerini doğrudan kullanabilmek için
            using Base = FilterBase<StateDim,MeasDim,CtrlDim>
            using Base::x_;
            using Base::P_;
            using Base::is_initialized_;

            // ilgili matris tiplerini tanımlayalım
            using StateMatrix = Eigen::Matrix<double,StateDim,StateDim>;
            using ControlMatrix = Eigen::Matrix<double,StateDim,CtrlDim>;
            using MeasurementMatrix = Eigen::Matrix<double,MeasDim,StateDim>;
            using MeasurementCovarianceMatrix = Eigen::Matrix<double,MeasDim,MeasDim>;

            KalmanFilter() = default;
            ~KalmanFilter() override = default;

            // --- Functions ---
            /**
             *@brief Sistemin matrislerini ayarlar (F,B,H,Q,R) 
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
                // Identity matrisini önceden hesaplayıp saklamak performansı artırır
                I_ = StateMatrix::Identity();
            }
            
            // Tahmin Adımı
            void predict() override
            {
                // Kontrol girdisi olmadan tahmin
                ControlVector<CtrlDim> u = ControlVector<CtrlDim>::Zero();
                predict(u);
            }
            
            void predict(const ControlVector<CtrlDim>& u) override
            {
                if(!is_initialized_) return;
                
                // x = F*x + B*u
                x_ = F_ * x_ + B_ * u;

                // P = F*P*F^T + Q
                P_ = F_*P_ * F_.transpose() + Q_;
            }
            
            // Güncelleme Adımı
            void update(const MeasurementVector<MeasDim>& z) override
            {
                if(!is_initialized_) return;
                
                // y = z - H*x 
                MeasurementVector<MeasDim> y = z - H_ * x_;

                // S = H*P*H^T +R
                MeasurementCovarianceMatrix S = H_ * P_ * H_.transpose() + R_;

                // K = P*H^T*S^-1 (Kalman Gain)
                // Modern C++ ve Eigen ile ters alma işlemi S.inverse() ile yapılır.
                // LLT (Cholesky) veya LU ayrıştırması daha hızlı ve güvenlidir, ancak S küçükse inverse() yeterlidir.
                Eigen::Matrix<double,StateDim,MeasDim> K = P_ * H_.transpose() * S.inverse();
                
                // x = x + K*y
                x_ = x_ + K*y;

                // P = (I - K*H)*P
                // Joseph Formu sayısal stabilite için daha iyidir: P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
                // Ancak standart form daha hızlıdır. Şimdilik standart formu kullanıyoruz.
                P_ = (I_ - K * H_) * P_;
            }

        private:
            StateMatrix F_; //Sistem / Durum Geçiş Matriksi
            ControlMatrix B_; // Kontrol Matriksi
            MeasurementMatrix H_; // Ölçüm Matriksi
            StateMatrix Q_; // Süreç Gürültüsü (Process Noise) Matrixi
            MeasurementCovarianceMatrix R_; // Ölçüm Gürültüsü (Measurement Noise) Matrixi

            StateMatrix I_; // Birim (Idenity) Matrix

    }; // class KalmanFilter


} // namespace kfl 