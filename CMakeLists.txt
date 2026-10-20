cmake_minimum_required(VERSION 3.16)
project(kalman_filter_lib VERSION 0.1.0 LANGUAGES CXX)

#Modern C++ Standards (C++17 or 20)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

#Tests and examples are built by default only when this is the top-level project
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(KFL_IS_TOP_LEVEL ON)
else()
    set(KFL_IS_TOP_LEVEL OFF)
endif()
option(KFL_BUILD_TESTS "Build unit tests (requires GTest)" ${KFL_IS_TOP_LEVEL})
option(KFL_BUILD_EXAMPLES "Build usage examples" ${KFL_IS_TOP_LEVEL})

#find Eigen3 library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#Library definition
#Filters are templated on compile-time dimensions, so the library is header-only (INTERFACE).
add_library(${PROJECT_NAME} INTERFACE)
add_library(kfl::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

#Set target include directions
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

#Let's bind the Eigen3 dependency to the library.
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

#Let's enable warnings during compilation (very important for clean code).
#Applied to our own tests/examples, not forced on library users.
add_library(kfl_warnings INTERFACE)
if(MSVC)
    target_compile_options(kfl_warnings INTERFACE /W4)
else()
    target_compile_options(kfl_warnings INTERFACE -Wall -Wextra -Wpedantic)
endif()

if(KFL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(KFL_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

#Install rules: lets other projects use find_package(kalman_filter_lib)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE kfl::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
    ARCH_INDEPENDENT
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
