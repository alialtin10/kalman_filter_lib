# project file architecture

``` bash
kalman_filter_lib/
├── CMakeLists.txt              # Projenin derleme ve bağlama kuralları
├── README.md                   # Proje dökümantasyonu
├── include/                    # Dışa aktarılan (public) başlık dosyaları
│   └── kalman_filter_lib/
│       ├── core/               # Temel arayüzler ve veri yapıları
│       │   ├── filter_base.hpp # Tüm filtrelerin türeyeceği sanal sınıf
│       │   └── types.hpp       # Eigen typedef'leri ve ortak yapılar
│       ├── models/             # Sistem ve Ölçüm modelleri
│       │   ├── system_model.hpp
│       │   └── measurement_model.hpp
│       ├── filters/            # Filtre algoritmalarının kendisi
│       │   ├── kf.hpp          # Standart Kalman Filtresi
│       │   ├── ekf.hpp         # Genişletilmiş Kalman Filtresi (Extended)
│       │   └── ukf.hpp         # Unscented Kalman Filtresi
│       └── utils/              # Matematiksel yardımcı fonksiyonlar
│           └── math_utils.hpp
├── src/                        # Implementasyon dosyaları (.cpp)
│   ├── models/
│   │   ├── system_model.cpp
│   │   └── measurement_model.cpp
│   └── filters/
│       ├── kf.cpp
│       ├── ekf.cpp
│       └── ukf.cpp
├── tests/                      # GTest ile yazılacak birim testleri (Unit Tests)
│   ├── test_kf.cpp
│   ├── test_ekf.cpp
│   └── CMakeLists.txt
└── examples/                   # Kütüphanenin kullanım örnekleri
    ├── 1d_robot_tracking.cpp   # Basit bir örnek
    └── 2d_differential_kinematics.cpp # Daha kompleks, robotik uyumlu bir örnek

```
