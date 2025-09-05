#pragma once
#include <Eigen/Dense>

template<int StateDim, int MeasurementDim>
class MeasurementModelBase
{
public:
    using StateVector = Eigen::Matrix<double, StateDim, 1>;
    using MeasurementVector = Eigen::Matrix<double, MeasurementDim, 1>;
    using MeasurementMatrix = Eigen::Matrix<double, MeasurementDim, MeasurementDim>;
    using CrossCovMatrix = Eigen::Matrix<double, MeasurementDim, StateDim>;

    virtual ~MeasurementModelBase() = default;
    virtual MeasurementVector h(const StateVector& x) const = 0;
    virtual CrossCovMatrix jacobian(const StateVector& x) const = 0;
    virtual MeasurementMatrix MeasNoise(const double dtime) const = 0;
};
