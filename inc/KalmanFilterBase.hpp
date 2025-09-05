#pragma once
#include <Eigen/Dense>

template<int StateDim, int MeasurementDim>
class KalmanFilterBase {
public:
    using StateVector = Eigen::Matrix<double, StateDim, 1>;
    using StateMatrix = Eigen::Matrix<double, StateDim, StateDim>;
    using MeasurementVector = Eigen::Matrix<double, MeasurementDim, 1>;
    using MeasurementMatrix = Eigen::Matrix<double, MeasurementDim, MeasurementDim>;

    virtual ~KalmanFilterBase() = default;

    virtual void initialize(const StateVector& x0, const StateMatrix& P0) = 0;

    virtual void predict(const StateVector& u, const unsigned int dtime) = 0;
    virtual void update(const MeasurementVector& z) = 0;

    virtual const StateVector& state() const = 0;
};
