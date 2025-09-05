#pragma once
#include <IMotionModel.hpp>
#include <Eigen/Dense>

template<int StateDim, int MeasurementDim>
class MotionModelBase : public IMotionModel
{
public:

    using StateVector = Eigen::Matrix<double, StateDim, 1>;
    using StateMatrix = Eigen::Matrix<double, StateDim, StateDim>;

    virtual ~MotionModelBase() = default;
    virtual StateMatrix ProcessNoise () = 0;
    virtual StateVector f(const StateVector& x, const StateVector& u, double dt) = 0;
    virtual StateMatrix jacobian(const StateVector& x, const StateVector& u, double dt) = 0;
};
