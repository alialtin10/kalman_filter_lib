#pragma once

#include "types.hpp"
#include "../utils/math_utils.hpp"

#include <Eigen/Cholesky>
#include <limits>
#include <stdexcept>

namespace kfl
{

/**
 * @brief Kalman Filtresi türevleri için ortak taban sınıf.
 *
 * Durum (x) ve kovaryansı (P) tutar. Ölçüm boyutu sınıf parametresi değildir; böylece aynı filtre
 * farklı boyutlu sensörlerle (GPS, pusula, LiDAR landmark vb.) sırayla güncellenebilir.
 *
 * @tparam StateDim Durum vektörünün boyutu (Nx1)
 */
template <int StateDim>
class FilterBase {
    public:
        //C++11 sanal yıkıcı (Bellek sızıntılarını önlemek için)
        virtual ~FilterBase() = default;

        /**
         * @brief Filtrenin durumunu başlatır
         * @param initial_state Başlangıç durum vektörü
         * @param initial_covariance Başlangıç belirsizlik (kovaryans) matrixi
         */
        virtual void initialize(const StateVector<StateDim>& initial_state,
                                const StateCovariance<StateDim>& initial_covariance)
        {
            x_ = initial_state;
            P_ = initial_covariance;
            last_nis_ = 0.0;
            is_initialized_ = true;
        }

        /**
         * @brief Filtreyi başlatılmamış duruma döndürür.
         */
        void reset() { is_initialized_ = false; }

         // --- Getter Fonksiyonları (Encapsulation) ---

         const StateVector<StateDim>& getState() const {return x_;}
         const StateCovariance<StateDim>& getCovariance() const {return P_;}
         bool isInitialized() const {return is_initialized_;}

         /**
          * @brief Son güncellemenin NIS (Normalized Innovation Squared) değeri: y^T S^-1 y
          * Tutarlı bir filtrede ölçüm boyutu kadar serbestlik dereceli ki-kare dağılımına uyar.
          */
         double getLastNIS() const {return last_nis_;}

         /// Gating kullanılmadığında update() fonksiyonlarına verilen varsayılan eşik
         static constexpr double kNoGate = std::numeric_limits<double>::infinity();

    protected:
        void requireInitialized() const
        {
            if (!is_initialized_)
                throw std::logic_error("kfl: filtre initialize() çağrılmadan kullanıldı");
        }

        /**
         * @brief KF ve EKF'nin ortak düzeltme adımı (Joseph formu).
         * @param y   İnovasyon (ölçüm artığı) z - h(x)
         * @param H   Ölçüm Jacobian'ı
         * @param R   Ölçüm gürültüsü kovaryansı
         * @param nis_gate NIS bu eşiği aşarsa ölçüm reddedilir (aykırı değer eleme)
         * @return Ölçüm kabul edildiyse true
         */
        template <int MeasDim>
        bool josephUpdate(const MeasurementVector<MeasDim>& y,
                          const Matrix<MeasDim,StateDim>& H,
                          const MeasurementCovariance<MeasDim>& R,
                          double nis_gate)
        {
            // S = H*P*H^T + R
            const MeasurementCovariance<MeasDim> S = utils::symmetrize(H * P_ * H.transpose() + R);

            // S^-1 doğrudan alınmaz; Cholesky (LLT) hem daha hızlı hem sayısal olarak daha güvenlidir.
            const Eigen::LLT<MeasurementCovariance<MeasDim>> llt(S);
            if (llt.info() != Eigen::Success)
                throw std::runtime_error("kfl: inovasyon kovaryansı S pozitif tanımlı değil");

            last_nis_ = y.dot(llt.solve(y));
            if (last_nis_ > nis_gate) return false;

            // K = P*H^T*S^-1  <=>  K^T = S^-1*H*P   (P ve S simetrik)
            const Matrix<StateDim,MeasDim> K = llt.solve(H * P_).transpose();

            // x = x + K*y
            x_ += K * y;

            // Joseph formu: P = (I-KH)*P*(I-KH)^T + K*R*K^T
            // Standart forma göre biraz daha pahalı ama P'nin simetrik ve pozitif tanımlı kalmasını sağlar.
            const StateCovariance<StateDim> I_KH = StateCovariance<StateDim>::Identity() - K * H;
            P_ = utils::symmetrize(I_KH * P_ * I_KH.transpose() + K * R * K.transpose());
            return true;
        }

        // Sadece türetilen sınıflar erişebilsin
        StateVector<StateDim> x_ = StateVector<StateDim>::Zero(); // Durum vektörü (x)
        StateCovariance<StateDim> P_ =StateCovariance<StateDim>::Identity(); // Kovaryans Matrixi (P)
        double last_nis_ = 0.0;
        bool is_initialized_ = false;
};


} // namespace kfl
