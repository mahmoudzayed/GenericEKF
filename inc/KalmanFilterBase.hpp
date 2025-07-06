#pragma once
#include <Eigen/Dense>

template<int StateDim, int MeasurementDim>
class KalmanFilterBase {
public:
    using StateVector = Eigen::Matrix<double, StateDim, 1>;
    using StateMatrix = Eigen::Matrix<double, StateDim, StateDim>;
    using MeasurementVector = Eigen::Matrix<double, MeasurementDim, 1>;
    using MeasurementMatrix = Eigen::Matrix<double, MeasurementDim, MeasurementDim>;
    using CrossCovMatrix = Eigen::Matrix<double, MeasurementDim, StateDim>;

    virtual ~KalmanFilterBase() = default;

    virtual void initialize(const StateVector& x0, const StateMatrix& P0,
                            const StateMatrix& Q, const MeasurementMatrix& R) = 0;

    virtual void predict() = 0;
    virtual void update(const MeasurementVector& z) = 0;

    virtual StateVector state() const = 0;
};