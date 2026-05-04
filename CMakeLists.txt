cmake_minimum_required(VERSION 3.16)
project(kalman_filter_lib VERSION 1.0.0 LANGUAGES CXX)

#Modern C++ Standards (C++17 or 20)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#find Eigen3 library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#Library definition (Interface or Static/Shared)
#Since it will be template-heavy, we can start with OBJECT or INTERFACE for now.
#However, we are making it a static library because we will put some implementations in src/ later.
add_library(${PROJECT_NAME} STATIC
    include/kalman_filter_lib/core/filter_base.hpp
    include/kalman_filter_lib/core/types.hpp
)

#Set target include directions
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

#Let's bind the Eigen3 dependency to the library.
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

#Let's enable warnings during compilation (very important for clean code).
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# An optional subdirectory for Examples
# We will add tests and examples here later
# add_subdirectory(examples)