#pragma once

#include <MotionModelBase.hpp>

class CVModel2D : public MotionModelBase<4, 2>
{
public:
    using Base = MotionModelBase<4, 2>;
    using typename Base::StateVector;
    using typename Base::StateMatrix;

    CVModel2D() = default;

    StateVector f(const StateVector& x, const StateVector& u, double dt)override;
    StateMatrix jacobian(const StateVector& x, const StateVector& u, double dt) override;
    StateMatrix ProcessNoise() override;
};
