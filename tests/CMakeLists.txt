find_package(GTest REQUIRED)

add_executable(kfl_tests
    test_utils.cpp
    test_kf.cpp
    test_ekf.cpp
    test_ukf.cpp
)

if(TARGET GTest::gtest_main)
    set(KFL_GTEST_MAIN GTest::gtest_main)
else()
    set(KFL_GTEST_MAIN GTest::Main)
endif()

target_link_libraries(kfl_tests PRIVATE kfl::kalman_filter_lib kfl_warnings ${KFL_GTEST_MAIN})

include(GoogleTest)
gtest_discover_tests(kfl_tests)
