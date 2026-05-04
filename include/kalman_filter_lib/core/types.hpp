#pragma once

#include <Eigen/Core>


namespace kfl{ //kalman_filter_lib

/**
 * @brief Eigen matrix types that are dimensioned (statically) at compile time.
 * @tparam Dim State vector size
 */
template <int Dim>
using StateVector = Eigen::Matrix<double,Dim,1>; 

template <int Dim>
using StateCovariance = Eigen::Matrix<double,Dim,Dim>;

/**
 * @brief Measurement vector and covariance types
 * @tparam MeasDim measurement vector size
 */
template <int MeasDim>
using MeasurementVector = Eigen::Matrix<double,MeasDim,1>;

template <int MeasDim>
using MeasurementCovariance = Eigen::Matrix<double,MeasDim,MeasDim>;

/**
 * @brief Control (input) vector type
 * @tparam CtrlDim control input size
 */
template <int CtrlDim>
using ControlVector = Eigen::Matrix<double,CtrlDim,1>;

} // namespace kfl