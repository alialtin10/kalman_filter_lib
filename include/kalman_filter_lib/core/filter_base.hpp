#pragma once 

#include "types.hpp"
#include <stdexcept>

namespace kfl
{

/**
 * @brief Kalman Filtresi türevleri için soyut taban sınıf.
 * @tparam StateDim Durum vektörünün boyutu (Nx1)
 * @tparam MeasDim Ölçüm vektörünün boyutu (Mx1)
 * @tparam CtrlDim Kontrol girdisinin boyutu (Ux1), varsayılan 0
 */
template <int StateDim, int MeasDim, int CtrlDim = 0>
class FilterBase {
    public:
        //C++11 sanal yıkıcı (Bellek sızıntılarını önmlemek için)
        virtual ~FilterBase() = default;

        /**
         * @brief Filtrenin durumunu başlatır
         * @param initial_state Başlangıç durum vektörü
         * @param initial_covariance Başlangıç belirsizlik (kovaryans) matrixi
         */
        virtual void initialize(const StateVector<StateDim>& initial_state
                                const StateCovariance<StateDim>& initial_covariance)
        {
            x_ = initial_state;
            P_ = initial_covariance;
            is_initialized_ = true;
        }

        /**
         * @brief Tahmin (Predict) adımı (Kontrol girdisi olmadan)
         */
        virtual void predict() = 0;

        /**
         * @brief Tahmin (Predict) adımı (Kontrol girdisi ile)
         * @param u Kontrol girdisi (Örn: Hız, Açısal hız)
         */
        virtual void predict(const ControlVector<CtrlDim>& u) = 0;

        /**
         * @brief Güncelleme (Update) adımı - Durumu sensör verisiyle düzeltir.
         * @param z sensör ölçm vektörü
         */
         virtual void update(const MeasurementVector<MeasDim>& z) = 0;

         // --- Getter ve Setter Fonksiyonları (Encapsulation) ---

         const StateVector<StateDim>& getState() const {return x_;}
         const StateCovariance<StateDim>& getCovariance() const {return P_;}
         bool isInitialized() const {return is_initialized_;}
    
    protected:
        // Sadece türetilen sınıflar erişebilsin
        StateVector<StateDim> x_ = StateVector<StateDim>::Zero(); // Durum vektörü (x)
        StateCovariance<StateDim> P_ =StateCovariance<StateDim>::Identity(); // Kovaryans Matrixi (P)
        bool is_initialized_ = false;
};


} // namespace kfl